#!/usr/bin/env python3
"""CI gate: firmware must fit the STM32F103C6T6 Flash and SRAM budgets.

Parses `arm-none-eabi-size` (Berkeley) output and fails the job when the
Flash or RAM usage exceeds the device capacity, so resource regressions are
caught before they reach hardware.
"""

import re
import sys

FLASH_BUDGET = 32768  # 32 KiB
RAM_BUDGET = 10240    # 10 KiB


def main():
    if len(sys.argv) != 2:
        sys.exit("usage: check_firmware_size.py <size output file>")

    with open(sys.argv[1], "r", encoding="utf-8") as handle:
        output = handle.read()

    # Only the first three columns (text/data/bss) are always decimal; the
    # dec/hex columns may start with a-f and must not be part of the match.
    match = re.search(
        r"^\s*(\d+)\s+(\d+)\s+(\d+)",
        output,
        re.MULTILINE,
    )
    if match is None:
        sys.exit("could not parse arm-none-eabi-size output")

    text_size, data_size, bss_size = (int(value) for value in match.groups())
    flash_used = text_size + data_size
    ram_used = data_size + bss_size

    print(
        "Flash: %d / %d B (%.2f%%)"
        % (flash_used, FLASH_BUDGET, 100.0 * flash_used / FLASH_BUDGET)
    )
    print(
        "RAM:   %d / %d B (%.2f%%)"
        % (ram_used, RAM_BUDGET, 100.0 * ram_used / RAM_BUDGET)
    )

    if flash_used > FLASH_BUDGET:
        sys.exit("ERROR: Flash budget exceeded")
    if ram_used > RAM_BUDGET:
        sys.exit("ERROR: RAM budget exceeded")

    print("Size budgets OK")


if __name__ == "__main__":
    main()
