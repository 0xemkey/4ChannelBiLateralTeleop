#!/usr/bin/env python3
"""Validate the physical Touch device topology used by OpenHaptics.

The HDAPI nodes open logical device names ("Left", "Default Device").  The
logical identity lives in ~/.3dsystems/config, and the USB/tty identity lives
in the udev-created stable links.  This checker intentionally validates those
stable identities instead of raw ttyACM enumeration order.
"""
from __future__ import annotations

import argparse
import os
import stat
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class ExpectedDevice:
    label: str
    config: Path
    channel: str
    config_serial: str
    stable_tty: str
    identity_link: str
    usb_path: str


def read_kv_config(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for raw_line in path.read_text().splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip()
    return values


def dev_path(dev_root: Path, absolute_dev: str) -> Path:
    path = Path(absolute_dev)
    if path.is_absolute():
        try:
            return dev_root / path.relative_to("/dev")
        except ValueError:
            pass
    return dev_root / path


def run_udevadm(device: Path) -> dict[str, str]:
    try:
        proc = subprocess.run(
            ["udevadm", "info", "-q", "property", "-n", str(device)],
            check=False,
            capture_output=True,
            text=True,
            timeout=3.0,
        )
    except (FileNotFoundError, subprocess.SubprocessError):
        return {}
    if proc.returncode != 0:
        return {}
    props: dict[str, str] = {}
    for line in proc.stdout.splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            props[key] = value
    return props


def is_read_write(path: Path) -> bool:
    return os.access(path, os.R_OK) and os.access(path, os.W_OK)


def check_one(expected: ExpectedDevice, dev_root: Path, allow_regular_files: bool) -> list[str]:
    failures: list[str] = []
    cfg = read_kv_config(expected.config)
    if cfg.get("Channel") != expected.channel:
        failures.append(
            f"{expected.label}: {expected.config} Channel={cfg.get('Channel')!r}, "
            f"expected {expected.channel}"
        )
    if cfg.get("DeviceSerialNumber") != expected.config_serial:
        failures.append(
            f"{expected.label}: {expected.config} DeviceSerialNumber="
            f"{cfg.get('DeviceSerialNumber')!r}, expected {expected.config_serial}"
        )

    tty = dev_path(dev_root, expected.stable_tty)
    identity = dev_path(dev_root, expected.identity_link)
    for label, path in (("stable channel link", tty), ("identity link", identity)):
        if not path.exists():
            failures.append(f"{expected.label}: missing {label}: {path}")
            continue
        try:
            mode = path.stat().st_mode
        except OSError as exc:
            failures.append(f"{expected.label}: cannot stat {label} {path}: {exc}")
            continue
        if not allow_regular_files and not stat.S_ISCHR(mode):
            failures.append(f"{expected.label}: {label} {path} is not a character device")
        if not is_read_write(path):
            failures.append(f"{expected.label}: {label} {path} is not readable and writable")

    if tty.exists() and identity.exists():
        try:
            if tty.resolve() != identity.resolve():
                failures.append(
                    f"{expected.label}: {tty} and {identity} resolve to different targets "
                    f"({tty.resolve()} vs {identity.resolve()})"
                )
        except OSError as exc:
            failures.append(f"{expected.label}: cannot resolve device links: {exc}")

    props = run_udevadm(tty) if dev_root == Path("/dev") else {}
    if props:
        if props.get("ID_VENDOR_ID") != "2988" or props.get("ID_MODEL_ID") != "0302":
            failures.append(
                f"{expected.label}: {tty} udev id is "
                f"{props.get('ID_VENDOR_ID')}:{props.get('ID_MODEL_ID')}, expected 2988:0302"
            )
        if props.get("ID_PATH") != expected.usb_path:
            failures.append(
                f"{expected.label}: {tty} ID_PATH={props.get('ID_PATH')!r}, "
                f"expected {expected.usb_path!r}"
            )

    return failures


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    parser.add_argument("--left-config", type=Path, required=True)
    parser.add_argument("--right-config", type=Path, required=True)
    parser.add_argument("--dev-root", type=Path, default=Path("/dev"))
    parser.add_argument(
        "--allow-regular-files",
        action="store_true",
        help="test-only escape hatch for fake device files",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    expected = [
        ExpectedDevice(
            label="Left",
            config=args.left_config,
            channel="300",
            config_serial="18019000358",
            stable_tty="/dev/ttyACM300",
            identity_link="/dev/touch_18019000358",
            usb_path="pci-0000:00:14.0-usb-0:10.1:1.0",
        ),
        ExpectedDevice(
            label="Default Device",
            config=args.right_config,
            channel="301",
            config_serial="18019000365",
            stable_tty="/dev/ttyACM301",
            identity_link="/dev/touch_18019000365",
            usb_path="pci-0000:00:14.0-usb-0:10.2:1.0",
        ),
    ]

    failures: list[str] = []
    for device in expected:
        if not device.config.is_file():
            failures.append(f"{device.label}: missing config {device.config}")
            continue
        if not os.access(device.config, os.R_OK):
            failures.append(f"{device.label}: config is not readable: {device.config}")
            continue
        failures.extend(check_one(device, args.dev_root, args.allow_regular_files))

    if failures:
        for failure in failures:
            print(f"[touch-device-check][error] {failure}", file=sys.stderr)
        return 1

    print("[touch-device-check] Left: config channel 300, stable tty /dev/ttyACM300, identity /dev/touch_18019000358")
    print("[touch-device-check] Default Device: config channel 301, stable tty /dev/ttyACM301, identity /dev/touch_18019000365")
    print("[touch-device-check] PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
