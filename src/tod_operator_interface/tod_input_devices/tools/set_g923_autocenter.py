#!/usr/bin/env python3
import argparse
import glob
import os
import pathlib
import struct


EV_FF = 0x15
FF_AUTOCENTER = 0x61
G923_EVENT_PATTERN = "/dev/input/by-id/*G923*-event-joystick"

_IOC_READ = 2
_IOC_DIRSHIFT = 30
_IOC_SIZESHIFT = 16
_IOC_TYPESHIFT = 8


def autocenter_value(percent: int) -> int:
    if not 0 <= percent <= 100:
        raise ValueError("strength must be between 0 and 100")
    return int(percent * 65535 / 100)


def encode_autocenter_event(percent: int) -> bytes:
    return struct.pack(
        "@llHHi", 0, 0, EV_FF, FF_AUTOCENTER, autocenter_value(percent)
    )


def resolve_g923_device() -> pathlib.Path:
    matches = [pathlib.Path(path) for path in glob.glob(G923_EVENT_PATTERN)]
    if len(matches) != 1:
        raise RuntimeError(
            f"expected exactly one G923 event device, found {len(matches)}"
        )
    return matches[0].resolve()


def _eviocgname(length: int) -> int:
    return (
        (_IOC_READ << _IOC_DIRSHIFT)
        | (length << _IOC_SIZESHIFT)
        | (ord("E") << _IOC_TYPESHIFT)
        | 0x06
    )


def read_device_name(device_fd: int) -> str:
    import fcntl

    name_buffer = bytearray(256)
    fcntl.ioctl(device_fd, _eviocgname(len(name_buffer)), name_buffer, True)
    return bytes(name_buffer).split(b"\0", 1)[0].decode("utf-8", errors="replace")


def set_autocenter(percent: int) -> tuple[pathlib.Path, str]:
    device = resolve_g923_device()
    device_fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
    try:
        device_name = read_device_name(device_fd)
        if "Logitech" not in device_name or "G923" not in device_name:
            raise RuntimeError(f"refusing non-G923 input device: {device_name}")
        os.write(device_fd, encode_autocenter_event(percent))
    finally:
        os.close(device_fd)
    return device, device_name


def main() -> int:
    parser = argparse.ArgumentParser(description="Set Logitech G923 autocenter strength")
    parser.add_argument("strength", nargs="?", type=int, default=30)
    args = parser.parse_args()

    try:
        device, device_name = set_autocenter(args.strength)
    except (OSError, RuntimeError, ValueError) as error:
        parser.error(str(error))

    print(f"Applied {args.strength}% autocenter to {device_name} ({device})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
