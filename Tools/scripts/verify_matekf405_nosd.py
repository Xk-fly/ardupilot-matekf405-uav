#!/usr/bin/env python3
"""Verify that the MatekF405 board is built without SPI SD/FATFS support."""

from __future__ import annotations

import argparse
import pickle
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HWDEF = ROOT / "libraries/AP_HAL_ChibiOS/hwdef/MatekF405/hwdef.dat"
BUILD_HWDEF = ROOT / "build/MatekF405/hwdef.h"
BUILD_ENV = ROOT / "build/MatekF405/env.py"


def active_lines(path: Path) -> list[str]:
    lines: list[str] = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if line and not line.startswith("#"):
            lines.append(line)
    return lines


def verify_source() -> None:
    lines = active_lines(HWDEF)
    assert not any(line.lower().startswith("spidev sdcard ") for line in lines), (
        "MatekF405 hwdef still registers an active SPI device named 'sdcard'"
    )
    assert "define HAL_OS_FATFS_IO 1" not in lines, (
        "MatekF405 hwdef still actively defines HAL_OS_FATFS_IO=1"
    )


def verify_generated_build() -> None:
    assert BUILD_HWDEF.is_file(), f"missing generated file: {BUILD_HWDEF}"
    assert BUILD_ENV.is_file(), f"missing generated file: {BUILD_ENV}"

    generated = BUILD_HWDEF.read_text(encoding="utf-8", errors="replace")
    forbidden = [
        "#define USE_POSIX",
        "#define HAL_USE_MMC_SPI TRUE",
        "#define HAL_SDCARD_SPI_HOOK TRUE",
        "#define HAL_OS_FATFS_IO 1",
    ]
    for marker in forbidden:
        assert marker not in generated, f"generated hwdef still contains: {marker}"

    with BUILD_ENV.open("rb") as stream:
        env = pickle.load(stream)
    assert str(env.get("WITH_FATFS", "0")) != "1", "WITH_FATFS is still enabled"
    flags = str(env.get("CHIBIOS_BUILD_FLAGS", ""))
    assert "USE_FATFS=no" in flags, f"expected USE_FATFS=no, got: {flags}"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source-only",
        action="store_true",
        help="verify only the board source before a configure/build exists",
    )
    args = parser.parse_args()

    verify_source()
    if not args.source_only:
        verify_generated_build()
    print("PASS: MatekF405 SD/FATFS is disabled in source and requested checks")


if __name__ == "__main__":
    main()
