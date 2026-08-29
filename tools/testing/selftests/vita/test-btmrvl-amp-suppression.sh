#!/bin/sh
# SPDX-License-Identifier: GPL-2.0
set -eu

CDPATH=
export CDPATH
root=$(cd -- "$(dirname -- "$0")/../../../.." && pwd)
driver="$root/drivers/bluetooth/btmrvl_sdio.c"
dtsi="$root/arch/arm/boot/dts/vita.dtsi"
defconfig="$root/arch/arm/configs/vita_defconfig"

python3 - "$driver" "$dtsi" "$defconfig" <<'PY'
from pathlib import Path
import re
import sys

driver = Path(sys.argv[1]).read_text()
dtsi = Path(sys.argv[2]).read_text()
defconfig = Path(sys.argv[3]).read_text()

probe_start = driver.index("static int btmrvl_sdio_probe(")
probe_end = driver.index("static void btmrvl_sdio_remove(", probe_start)
probe = driver[probe_start:probe_end]

availability = "of_device_is_available(func->dev.of_node)"
if availability not in probe:
    raise SystemExit("FAIL: btmrvl SDIO probe does not honor a disabled OF function")
if probe.index(availability) > probe.index("btmrvl_sdio_register_dev(card)"):
    raise SystemExit("FAIL: OF availability check runs after device registration")
if probe.index(availability) > probe.index("btmrvl_sdio_download_fw(card)"):
    raise SystemExit("FAIL: OF availability check runs after firmware access")

def block_after(text, marker):
    start = text.find(marker)
    if start < 0:
        raise SystemExit(f"FAIL: cannot locate {marker}")
    opening = text.find("{", start)
    depth = 0
    for pos in range(opening, len(text)):
        if text[pos] == "{":
            depth += 1
        elif text[pos] == "}":
            depth -= 1
            if depth == 0:
                return text[opening + 1:pos]
    raise SystemExit(f"FAIL: unterminated block for {marker}")

sdif2 = block_after(dtsi, "sdif2: mmc@e0c10000")
amp_body = block_after(sdif2, "bluetooth-amp@3")
if not re.search(r"reg\s*=\s*<(?:0x)?0*3>\s*;", amp_body):
    raise SystemExit("FAIL: Bluetooth AMP child is not SDIO function 3")
if not re.search(r'status\s*=\s*"disabled"\s*;', amp_body):
    raise SystemExit("FAIL: Bluetooth AMP function 3 is not disabled")

for symbol in (
    "CONFIG_BT_MRVL=y",
    "CONFIG_BT_MRVL_SDIO=y",
    "CONFIG_LEDS_CLASS_MULTICOLOR=y",
    "CONFIG_HID_PLAYSTATION=y",
):
    if not re.search(rf"^{re.escape(symbol)}$", defconfig, re.M):
        raise SystemExit(f"FAIL: Vita defconfig lacks {symbol}")

print("PASS: Vita disables only SD8787 AMP function 3 before btmrvl firmware access")
print("PASS: Marvell Bluetooth, SDIO transport, and PlayStation HID remain enabled")
PY
