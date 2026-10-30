/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include <math.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "commander.h"
#include "crtp_commander.h"
#include "crtp_commander_high_level.h"

#include "cf_math.h"
#include "param.h"
#include "log.h"
#include "num.h"
#include "physicalConstants.h"
#include "stm32_legacy.h"
#include "static_mem.h"
#include "system.h"
#define DEBUG_MODULE "CMDR"
#include "debug_cf.h"

static bool isInit;
const static setpoint_t nullSetpoint;
static setpoint_t tempSetpoint;
static state_t lastState;
const static int priorityDisable = COMMANDER_PRIORITY_DISABLE;

static uint32_t lastUpdate;
static bool enableHighLevel = false;
static bool autoArmed = false;

// Gentle landing (manual thrust mode only), tunable live via the "land" params.
// When the throttle drops to zero while flying, or the controller link is lost, the drone does
// not cut its motors. Instead it descends at landSpeed: the vertical speed is integrated from the
// accelerometer and a PI loop around the learned hover thrust holds the descent. Touchdown is
// seen when the descent stops while the loop is still asking to go down; then the motors stop.
// Raising the throttle again at any time hands control straight back to the pilot.
#define LAND_FLYING_THRUST   15000   // thrust above this counts as flying
#define LAND_ARM_TIME        1.0f    // s of flying before a throttle drop triggers a landing
#define LAND_GROUND_TIME     0.3f    // s below LAND_FLYING_THRUST that means "on the ground"
#define LAND_SPEEDUP_TIME    0.5f    // s to ramp the descent target from 0 to landSpeed
#define LAND_TOUCHDOWN_TIME  0.2f    // s the descent must be stopped before cutting the motors
#define LAND_MAX_THRUST      60000.0f
#define LAND_IMPACT_FILTER   0.02f   // s, low-pass on vertical acc for impact detection
#define LAND_AVERAGE_TIME    2.0f    // s, averaging time for hoverThrust and accBias
#define LAND_VZ_LEAK_TIME    4.0f    // s, the speed estimate forgets old acceleration (bounds drift)

static uint8_t landEnable = 1;
static float landSpeed = 0.3f;       // m/s target descent speed
static float landKp = 20000.0f;      // thrust per m/s of speed error
static float landKi = 15000.0f;      // thrust per m of accumulated speed error
static float landMaxTime = 8.0f;     // s, after this the thrust just ramps down
static float landImpactG = 0.15f;    // g, an upward jolt this big while landing means touchdown

static bool landing;
static float hoverThrust;            // low-pass of the pilot's thrust while flying
static float lastFlyingThrust;       // the pilot's last thrust before the throttle dropped
static float accBias;                // low-pass of vertical acc while flying (should average 0)
static float flyTime, lowTime;
static float landTime, landVz, landI, landThrust, touchdownTime;
static bool landWasDescending;
static float landAccFiltered, landAccPeak;
static uint32_t landLastTick;

static QueueHandle_t setpointQueue;
STATIC_MEM_QUEUE_ALLOC(setpointQueue, 1, sizeof(setpoint_t));
static QueueHandle_t priorityQueue;
STATIC_MEM_QUEUE_ALLOC(priorityQueue, 1, sizeof(int));

/* Public functions */
void commanderInit(void)
{
  setpointQueue = STATIC_MEM_QUEUE_CREATE(setpointQueue);
  ASSERT(setpointQueue);
  xQueueSend(setpointQueue, &nullSetpoint, 0);

  priorityQueue = STATIC_MEM_QUEUE_CREATE(priorityQueue);
  ASSERT(priorityQueue);
  xQueueSend(priorityQueue, &priorityDisable, 0);

  crtpCommanderInit();
  crtpCommanderHighLevelInit();
  lastUpdate = xTaskGetTickCount();

  isInit = true;
}

void commanderSetSetpoint(setpoint_t *setpoint, int priority)
{
  int currentPriority;

  const BaseType_t peekResult = xQueuePeek(priorityQueue, &currentPriority, 0);
  ASSERT(peekResult == pdTRUE);

  if (priority >= currentPriority) {
    // Auto-arm: a controller is sending setpoints and its throttle is at zero.
    // Disarming happens in commanderGetSetpoint() when the setpoints stop.
    // Arming is refused while blocked, e.g. while an OTA update is being written.
    if (!autoArmed && setpoint->thrust == 0 && !systemIsArmingBlocked()) {
      autoArmed = true;
      systemSetArmed(true);
      DEBUG_PRINTI("Armed (controller connected, throttle at zero)\n");
    }
    setpoint->timestamp = xTaskGetTickCount();
    // This is a potential race but without effect on functionality
    xQueueOverwrite(setpointQueue, setpoint);
    xQueueOverwrite(priorityQueue, &priority);
    // Send the high-level planner to idle so it will forget its current state
    // and start over if we switch from low-level to high-level in the future.
    crtpCommanderHighLevelStop();
  }
}

void commanderNotifySetpointsStop(int remainValidMillisecs)
{
  uint32_t currentTime = xTaskGetTickCount();
  int timeSetback = MIN(
    COMMANDER_WDT_TIMEOUT_SHUTDOWN - M2T(remainValidMillisecs),
    currentTime
  );
  xQueuePeek(setpointQueue, &tempSetpoint, 0);
  tempSetpoint.timestamp = currentTime - timeSetback;
  xQueueOverwrite(setpointQueue, &tempSetpoint);
  crtpCommanderHighLevelTellState(&lastState);
}

static void stopLanding(const char *reason)
{
  landing = false;
  flyTime = 0;
  DEBUG_PRINTI("Landing ended: %s\n", reason);
}

// Replaces a zero thrust setpoint with a controlled descent, see the comment at the top.
static void gentleLanding(setpoint_t *setpoint, const state_t *state, bool linkLost)
{
  uint32_t now = xTaskGetTickCount();
  float dt = (float)(now - landLastTick) / configTICK_RATE_HZ;
  landLastTick = now;
  if (dt > 0.05f) {
    dt = 0.0f;  // first call or a stall: don't integrate a bogus step
  }
  // dt is 0 when two loop iterations fall in the same tick; the landing thrust must still be
  // applied then, otherwise the motors get a zero-thrust blip

  if (!systemIsArmed() || setpoint->mode.z != modeDisable) {
    landing = false;
    flyTime = 0;
    return;
  }

  float thrustIn = setpoint->thrust;

  if (landing && thrustIn > 0) {
    stopLanding("pilot took over");
  }

  if (!landing) {
    if (thrustIn >= LAND_FLYING_THRUST) {
      // While the pilot holds height the average vertical acceleration is zero, so whatever the
      // accelerometer reads on average is offset (motor vibration, drift since boot calibration)
      if (flyTime == 0) {
        hoverThrust = thrustIn;
        accBias = state->acc.z;
      } else {
        hoverThrust += (thrustIn - hoverThrust) * dt / LAND_AVERAGE_TIME;
        accBias += (state->acc.z - accBias) * dt / LAND_AVERAGE_TIME;
      }
      lastFlyingThrust = thrustIn;
      flyTime += dt;
      lowTime = 0;
    } else if (thrustIn == 0 && landEnable && flyTime >= LAND_ARM_TIME) {
      landing = true;
      // Never start above what the motors were last doing, so letting go can't speed them up
      hoverThrust = fminf(hoverThrust, lastFlyingThrust);
      landTime = 0;
      landVz = 0;  // the pilot was roughly holding height
      landI = 0;
      landThrust = hoverThrust;
      touchdownTime = 0;
      landWasDescending = false;
      landAccFiltered = 0;
      landAccPeak = 0;
      DEBUG_PRINTI("Landing gently (hover thrust %d)\n", (int)hoverThrust);
    } else {
      // Low but not zero throttle: still flying for a moment, on the ground if it stays low
      lowTime += dt;
      if (lowTime >= LAND_GROUND_TIME) {
        flyTime = 0;
      }
    }
  }

  if (!landing) {
    return;
  }

  landTime += dt;
  // acc.z: world vertical, gravity removed, in g
  float accZ = state->acc.z - accBias;
  landVz += accZ * GRAVITY_MAGNITUDE * dt;
  landAccFiltered += (accZ - landAccFiltered) * fminf(dt / LAND_IMPACT_FILTER, 1.0f);
  landAccPeak = fmaxf(landAccPeak, landAccFiltered);
  landVz -= landVz * dt / LAND_VZ_LEAK_TIME;

  if (landTime < landMaxTime) {
    float target = -landSpeed * fminf(landTime / LAND_SPEEDUP_TIME, 1.0f);
    float error = target - landVz;
    landI += landKi * error * dt;
    landI = constrain(landI, -hoverThrust, 0.0f);
    // Landing only ever takes thrust away: at most hover thrust, which stops a fall from speeding up
    landThrust = fminf(hoverThrust + landKp * error + landI, hoverThrust);

    // Touchdown: it was descending, now it has stopped although the loop still asks to go down
    if (landVz < -0.5f * landSpeed) {
      landWasDescending = true;
    }
    bool stopped = landWasDescending && landVz > -0.3f * landSpeed && landThrust < hoverThrust;
    touchdownTime = stopped ? touchdownTime + dt : 0;
    if (touchdownTime >= LAND_TOUCHDOWN_TIME || landThrust < 0.5f * hoverThrust) {
      stopLanding("touchdown");
      return;
    }
    // Impact: landing thrust never exceeds hover, so in the air the drone cannot accelerate
    // upwards; a sharp upward jolt can only be the ground pushing back
    if (landAccFiltered > landImpactG) {
      stopLanding("impact");
      return;
    }
  } else {
    // Taking too long (e.g. speed estimate drifted): sink by ramping the thrust down
    landThrust -= hoverThrust / 3.0f * dt;
    if (landThrust <= 0.0f) {
      stopLanding("timeout");
      return;
    }
  }

  setpoint->thrust = constrain(landThrust, 0.0f, LAND_MAX_THRUST);
  if (linkLost) {
    // Nobody is steering: stay level and keep the current heading
    setpoint->mode.roll = modeAbs;
    setpoint->mode.pitch = modeAbs;
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitude.roll = 0;
    setpoint->attitude.pitch = 0;
    setpoint->attitudeRate.yaw = 0;
  }
}

void commanderGetSetpoint(setpoint_t *setpoint, const state_t *state)
{
  xQueuePeek(setpointQueue, setpoint, 0);
  lastUpdate = setpoint->timestamp;
  uint32_t currentTime = xTaskGetTickCount();
  bool linkLost = false;

  if ((currentTime - setpoint->timestamp) > COMMANDER_WDT_TIMEOUT_SHUTDOWN) {
    if (enableHighLevel) {
      crtpCommanderHighLevelGetSetpoint(setpoint, state);
    }
    if (!enableHighLevel || crtpCommanderHighLevelIsStopped()) {
      memcpy(setpoint, &nullSetpoint, sizeof(nullSetpoint));
      linkLost = true;
    }
  } else if ((currentTime - setpoint->timestamp) > COMMANDER_WDT_TIMEOUT_STABILIZE) {
    xQueueOverwrite(priorityQueue, &priorityDisable);
    // Leveling ...
    setpoint->mode.x = modeDisable;
    setpoint->mode.y = modeDisable;
    setpoint->mode.roll = modeAbs;
    setpoint->mode.pitch = modeAbs;
    setpoint->mode.yaw = modeVelocity;
    setpoint->attitude.roll = 0;
    setpoint->attitude.pitch = 0;
    setpoint->attitudeRate.yaw = 0;
    // Keep Z as it is
  }

  gentleLanding(setpoint, state, linkLost);

  // Controller lost: once landed, disarm so it must send zero throttle again before the motors run
  if (linkLost && !landing && autoArmed) {
    autoArmed = false;
    systemSetArmed(false);
    DEBUG_PRINTW("Disarmed (no setpoints for 2 s)\n");
  }

  // This copying is not strictly necessary because stabilizer.c already keeps
  // a static state_t containing the most recent state estimate. However, it is
  // not accessible by the public interface.
  lastState = *state;
}

bool commanderTest(void)
{
  return isInit;
}

uint32_t commanderGetInactivityTime(void)
{
  return xTaskGetTickCount() - lastUpdate;
}

int commanderGetActivePriority(void)
{
  int priority;

  const BaseType_t peekResult = xQueuePeek(priorityQueue, &priority, 0);
  ASSERT(peekResult == pdTRUE);

  return priority;
}

PARAM_GROUP_START(commander)
PARAM_ADD(PARAM_UINT8, enHighLevel, &enableHighLevel)
PARAM_GROUP_STOP(commander)

PARAM_GROUP_START(land)
PARAM_ADD(PARAM_UINT8, enable, &landEnable)
PARAM_ADD(PARAM_FLOAT, speed, &landSpeed)
PARAM_ADD(PARAM_FLOAT, kp, &landKp)
PARAM_ADD(PARAM_FLOAT, ki, &landKi)
PARAM_ADD(PARAM_FLOAT, maxTime, &landMaxTime)
PARAM_ADD(PARAM_FLOAT, impactG, &landImpactG)
PARAM_GROUP_STOP(land)

LOG_GROUP_START(land)
LOG_ADD(LOG_UINT8, active, &landing)
LOG_ADD(LOG_FLOAT, vz, &landVz)
LOG_ADD(LOG_FLOAT, hover, &hoverThrust)
LOG_ADD(LOG_FLOAT, accBias, &accBias)
LOG_ADD(LOG_FLOAT, accPeak, &landAccPeak)
LOG_ADD(LOG_FLOAT, thrust, &landThrust)
LOG_GROUP_STOP(land)
