/**
 * Mender OTA updates for the drone, using the mender-mcu client (ESP-IDF >= 6.0).
 *
 * Mender only runs when the drone has joined a home network (station mode), since it
 * needs the internet. Flash writes stall the CPU, so for flight safety:
 *  - when a deployment starts downloading, arming is blocked, and if the drone is armed the
 *    download waits until it disarms (controller disconnected for 2 s)
 *  - arming stays blocked until the deployment succeeds or fails
 *  - the reboot into the new firmware also waits until the drone is disarmed
 * Polling the server and sending inventory are network-only and may happen in flight.
 *
 * Server and tenant token go in mender_credentials.h (not tracked by git), see
 * mender_credentials.h.example.
 */
#include "mender_ota.h"

#if __has_include("mender/client.h")

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_system.h"

#include "mender/client.h"
#include "mender/inventory.h"
#include "mender/esp-ota-update-module.h"

#include "system.h"
#include "wifi_esp32.h"
#define DEBUG_MODULE "MENDER"
#include "debug_cf.h"

#if __has_include("mender_credentials.h")
#include "mender_credentials.h"
#endif
#ifndef MENDER_SERVER_URL
#define MENDER_SERVER_URL "https://eu.hosted.mender.io"
#endif
#ifndef MENDER_TENANT_TOKEN
#define MENDER_TENANT_TOKEN ""
#endif

#define MENDER_DEVICE_TYPE      "esp-fly"
#define MENDER_WAIT_FOR_WIFI_MS 30000

static char mac_address[18];
static mender_identity_t identity = { .name = "mac", .value = mac_address };

static mender_err_t get_identity_cb(const mender_identity_t **id)
{
    *id = &identity;
    return MENDER_OK;
}

static mender_err_t network_connect_cb(void)
{
    return wifiStaIsConnected() ? MENDER_OK : MENDER_FAIL;
}

static mender_err_t network_release_cb(void)
{
    return MENDER_OK;
}

static mender_err_t deployment_status_cb(mender_deployment_status_t status, const char *desc)
{
    DEBUG_PRINTI("deployment status: %s\n", desc ? desc : "");
    switch (status) {
    case MENDER_DEPLOYMENT_STATUS_DOWNLOADING:
        // The client ignores our return value, so hold the download here until the drone is
        // disarmed. Arming is blocked first so it cannot re-arm once we let the download start.
        systemSetArmingBlocked(true);
        if (systemIsArmed()) {
            DEBUG_PRINTI("update waiting for the drone to disarm\n");
            while (systemIsArmed()) {
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
        break;
    case MENDER_DEPLOYMENT_STATUS_INSTALLING:
    case MENDER_DEPLOYMENT_STATUS_REBOOTING:
        systemSetArmingBlocked(true);
        break;
    case MENDER_DEPLOYMENT_STATUS_SUCCESS:
    case MENDER_DEPLOYMENT_STATUS_FAILURE:
    default:
        systemSetArmingBlocked(false);
        break;
    }
    return MENDER_OK;
}

static mender_err_t restart_cb(void)
{
    while (systemIsArmed()) {
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    DEBUG_PRINTI("restarting into the new firmware\n");
    esp_restart();
    return MENDER_OK;
}

static mender_err_t persistent_inventory_cb(mender_keystore_t **keystore, uint8_t *keystore_len)
{
    static mender_keystore_t inventory[] = { { .name = "firmware", .value = "esp-drone" },
                                             { .name = "board", .value = "ESP-FLY (XIAO ESP32-S3)" } };
    *keystore = inventory;
    *keystore_len = sizeof(inventory) / sizeof(inventory[0]);
    return MENDER_OK;
}

static bool waitForHomeNetwork(void)
{
    for (int waited = 0; waited < MENDER_WAIT_FOR_WIFI_MS; waited += 1000) {
        if (wifiStaIsConnected()) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    return false;
}

static void menderTask(void *arg)
{
    if (strlen(MENDER_TENANT_TOKEN) == 0) {
        DEBUG_PRINTW("no tenant token in mender_credentials.h, Mender disabled\n");
        vTaskDelete(NULL);
    }
    if (!waitForHomeNetwork()) {
        DEBUG_PRINTW("not on a home network, Mender disabled\n");
        vTaskDelete(NULL);
    }

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    snprintf(mac_address, sizeof(mac_address), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    mender_client_config_t config = {
        .device_type = MENDER_DEVICE_TYPE,
        .host = MENDER_SERVER_URL,
        .tenant_token = MENDER_TENANT_TOKEN,
    };
    mender_client_callbacks_t callbacks = {
        .network_connect = network_connect_cb,
        .network_release = network_release_cb,
        .deployment_status = deployment_status_cb,
        .restart = restart_cb,
        .get_identity = get_identity_cb,
    };

    DEBUG_PRINTI("starting client %s, server %s, identity mac=%s\n",
                 mender_client_version(), MENDER_SERVER_URL, mac_address);
    if (mender_client_init(&config, &callbacks) != MENDER_OK ||
        mender_esp_ota_register_update_module() != MENDER_OK ||
        mender_inventory_add_callback(persistent_inventory_cb, true) != MENDER_OK) {
        DEBUG_PRINTE("failed to initialise the Mender client\n");
        vTaskDelete(NULL);
    }

    if (mender_client_activate() != MENDER_OK) {
        DEBUG_PRINTE("failed to activate the Mender client\n");
    }
    vTaskDelete(NULL);
}

void menderOtaStart(void)
{
    xTaskCreate(menderTask, "mender", 6144, NULL, 2, NULL);
}

#else /* mender-mcu not available (ESP-IDF < 6.0) */

void menderOtaStart(void)
{
}

#endif
