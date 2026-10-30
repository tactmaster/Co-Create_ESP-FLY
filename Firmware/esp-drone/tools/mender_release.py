"""
Build, upload and deploy ESP-FLY firmware with Mender, using the pymenderai SDK
(https://github.com/tactmaster/pymenderai).

The Personal Access Token is read from MENDER_API_KEY in the environment or in the repo's
.env file (gitignored). It is never printed.

  python mender_release.py status                  # list devices and their auth status
  python mender_release.py accept                  # accept pending esp-fly devices
  python mender_release.py build  NAME             # ESPDrone.bin -> NAME.mender (needs mender-artifact in WSL)
  python mender_release.py upload NAME             # upload NAME.mender
  python mender_release.py deploy NAME             # deploy release NAME to all esp-fly devices
  python mender_release.py release NAME            # build + upload + deploy

Building needs the mender-artifact tool inside WSL (sudo apt install mender-artifact).
"""
import argparse
import os
import pathlib
import subprocess
import sys
import time

from mender_sdk import MenderSDKClient

SERVER = os.environ.get("MENDER_SERVER_URL", "https://eu.hosted.mender.io")
DEVICE_TYPE = "esp-fly"
BUILD_DIR = pathlib.Path(os.environ.get("ESP_FLY_BUILD_DIR", r"C:\Espressif\builds\esp-fly"))
REPO_ROOT = pathlib.Path(__file__).resolve().parents[3]


def api_key():
    key = os.environ.get("MENDER_API_KEY")
    env_file = REPO_ROOT / ".env"
    if not key and env_file.exists():
        for line in env_file.read_text().splitlines():
            line = line.strip()
            if line.startswith("export "):
                line = line[len("export "):]
            if line.startswith("MENDER_API_KEY="):
                key = line.split("=", 1)[1].strip().strip('"').strip("'")
    if not key:
        sys.exit("MENDER_API_KEY not set (environment or .env)")
    return key


def client():
    return MenderSDKClient(base_url=SERVER, api_token=api_key())


def identity(dev):
    return ", ".join(f"{k}={v}" for k, v in (dev.get("identity_data") or {}).items())


def inventory(c, device_id):
    inv = c.inventory.get_device_inventory(device_id)
    return {a.get("name"): a.get("value") for a in inv.get("attributes", [])}


def esp_fly_device_ids(c):
    """Accepted devices reporting device_type esp-fly (other devices on the account are left alone)."""
    return [d["id"] for d in all_devices(c, "accepted")
            if inventory(c, d["id"]).get("device_type") == DEVICE_TYPE]


def all_devices(c, status=None):
    devices = []
    page = 1
    while True:
        batch = c.devices.list(page=page, per_page=100)
        devices += batch
        if len(batch) < 100:
            break
        page += 1
    return [d for d in devices if status is None or d.get("status") == status]


def cmd_status(c, args):
    for d in all_devices(c):
        inv = inventory(c, d["id"]) if d.get("status") == "accepted" else {}
        print(f"{d['id']}  {d.get('status', '?'):10}  {identity(d):24}  "
              f"{inv.get('device_type', '-'):12}  {inv.get('artifact_name', '-')}")


def cmd_accept(c, args):
    # Pending devices have no inventory yet, so accept by the identity our firmware sends
    pending = [d for d in all_devices(c, "pending") if "mac" in (d.get("identity_data") or {})]
    if not pending:
        print("no pending devices")
    for d in pending:
        c.devices.accept(d["id"])
        print(f"accepted {identity(d)}")


def mender_path(name):
    return BUILD_DIR / f"{name}.mender"


def to_wsl(path):
    p = str(path).replace("\\", "/")
    return f"/mnt/{p[0].lower()}{p[2:]}"


def cmd_build(c, args):
    image = BUILD_DIR / "ESPDrone.bin"
    if not image.exists():
        sys.exit(f"{image} not found, build the firmware first")
    out = mender_path(args.name)
    subprocess.run(["wsl", "mender-artifact", "--compression", "none",  # mender-mcu cannot read compressed artifacts
                    "write", "module-image",
                    "-T", "esp-ota", "-t", DEVICE_TYPE, "-n", args.name,
                    "-f", to_wsl(image), "-o", to_wsl(out)], check=True)
    print(f"built {out}")


def cmd_upload(c, args):
    path = mender_path(args.name)
    with open(path, "rb") as f:
        try:
            c.artifacts.upload(f, description=f"ESP-FLY firmware {args.name}")
        except ValueError:
            pass  # the server answers 201 Created with an empty body, which the SDK tries to parse
    print(f"uploaded {path.name}")


def cmd_deploy(c, args):
    accepted = esp_fly_device_ids(c)
    if not accepted:
        sys.exit(f"no accepted {DEVICE_TYPE} devices to deploy to")
    dep = c.deployments.create(name=f"{args.name} {time.strftime('%Y-%m-%d %H:%M')}",
                               artifact_name=args.name, devices=accepted)
    print(f"deployment {dep.get('id')} to {len(accepted)} device(s)")


def cmd_release(c, args):
    cmd_build(c, args)
    cmd_upload(c, args)
    cmd_deploy(c, args)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("status")
    sub.add_parser("accept")
    for name in ("build", "upload", "deploy", "release"):
        sp = sub.add_parser(name)
        sp.add_argument("name", help="artifact/release name, e.g. esp-fly-1.1")
    args = p.parse_args()
    globals()[f"cmd_{args.cmd}"](client(), args)


if __name__ == "__main__":
    main()
