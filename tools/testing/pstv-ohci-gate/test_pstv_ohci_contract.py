#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0
"""Source-level lifecycle contract for the production PSTV OHCI driver.

A real HCD lifecycle cannot be exercised on the host, so this guards the
invariants whose violation is expensive on hardware: touching the companion
window before the paired EHCI is proven alive (one-way bus poisoning),
hijacking the EHCI device's drvdata, retaining locks as if they were
lifetime references, and hard-coding the address or interrupt instead of
using the described resources.

Checks are regex-based on purpose: they pin behaviour, not variable names.
"""
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
DRIVER = ROOT / "drivers/usb/host/pstv-ohci.c"
DTSI = ROOT / "arch/arm/boot/dts/vita.dtsi"

# (description, pattern) - each must appear in the driver.
REQUIRED = [
    ("creates its own HCD on its own platform device",
     r"usb_create_hcd\(\s*&\w+_hc_driver\s*,\s*dev\b"),
    ("checks the paired EHCI is bound before using its drvdata",
     r"device_is_bound\s*\(\s*&ehci_pdev->dev\s*\)"),
    ("defers instead of failing when EHCI is not ready",
     r"-EPROBE_DEFER"),
    ("takes a reference on the paired EHCI HCD",
     r"usb_get_hcd\s*\("),
    ("takes a reference on the EHCI root hub",
     r"usb_get_dev\s*\("),
    ("requires the EHCI hardware to be accessible",
     r"HCD_HW_ACCESSIBLE\s*\("),
    ("requires the EHCI root hub to be registered",
     r"rh_registered"),
    ("holds a runtime-PM reference on the root hub",
     r"pm_runtime_resume_and_get\s*\(\s*&\w+->dev\s*\)"),
    ("releases that runtime-PM reference",
     r"pm_runtime_put_sync_autosuspend\s*\("),
    ("verifies the EHCI frame index is advancing",
     r"FRINDEX"),
    ("creates a managed device link to the EHCI supplier",
     r"device_link_add\s*\(\s*dev\s*,\s*&ehci_pdev->dev"),
    ("marks that link autoremove + runtime PM",
     r"DL_FLAG_AUTOREMOVE_CONSUMER\s*\|\s*\n?\s*DL_FLAG_PM_RUNTIME"),
    ("publishes the EHCI companion relationship",
     r"hs_companion\s*=\s*&\w+->self"),
    ("clears the companion relationship on teardown",
     r"hs_companion\s*=\s*NULL"),
    ("keeps Sony's mask/HCR/control reset order",
     r"INTR_DISABLE[\s\S]{0,200}CMD_STATUS[\s\S]{0,200}"
     r"writel_relaxed\(0,\s*hcd->regs \+ \w*CONTROL"),
    ("still runs generic OHCI init from the reset override",
     r"return\s+ohci_setup\(hcd\)"),
    ("removes the HCD before VBUS drops at reboot",
     r"register_reboot_notifier"),
    ("orders that ahead of the VBUS notifier (priority > 256)",
     r"REBOOT_PRIORITY\s+257"),
    ("refuses system sleep rather than replaying an unproven reset",
     r"return\s+-EBUSY"),
]

# (description, pattern) - none may appear in the driver.
FORBIDDEN = [
    ("shares HCD state with the EHCI controller",
     r"usb_create_shared_hcd"),
    ("overwrites the EHCI device's drvdata",
     r"(dev_set_drvdata|platform_set_drvdata)\s*\(\s*&?ehci"),
    # The literal base may only appear as the identity constant, never as an
    # argument to a mapping or region call.
    ("hard-codes the register window instead of mapping the resource",
     r"(ioremap|request_mem_region|of_iomap)[a-z_]*\([^;]*0x[eE]40"),
    ("hard-codes or rewrites the interrupt number",
     r"(request_irq\s*\(\s*\d|args\.args\[\d\]\s*=|irq_create_of_mapping)"),
    ("retains a device lock past probe",
     r"->controller_locked|->hub_locked"),
    ("depends on the debugfs diagnostic gate",
     r"debugfs|pstv-ohci-gate"),
    ("gates on an empty bus, which would break keyboard-at-boot",
     r"usb_hub_find_child"),
]


def check(source):
    failures = []
    for description, pattern in REQUIRED:
        if not re.search(pattern, source):
            failures.append("missing: driver never " + description)
    for description, pattern in FORBIDDEN:
        match = re.search(pattern, source)
        if match:
            failures.append("forbidden: driver %s (%r)"
                            % (description, match.group(0)))
    return failures


def check_dt(dtsi):
    failures = []
    node = re.search(r"ohci@e40e0200\s*\{[^}]*\}", dtsi)
    if not node:
        return ["missing: no ohci@e40e0200 node in vita.dtsi"]
    body = node.group(0)
    for description, pattern in [
        ("compatible", r'compatible\s*=\s*"vita,pstv-ohci"'),
        ("register window", r"reg\s*=\s*<0xe40e0200 0x100>"),
        ("interrupt", r"interrupts\s*=\s*<GIC_SPI 113"),
        ("paired EHCI phandle", r"vita,ehci\s*=\s*<&ehci0>"),
    ]:
        if not re.search(pattern, body):
            failures.append("missing: DT node has no correct " + description)
    # Hardware enablement is a separate, reviewed change.
    if not re.search(r'status\s*=\s*"disabled"', body):
        failures.append(
            "forbidden: DT node is enabled before hardware validation")
    return failures


def main():
    if not DRIVER.exists():
        print("FAIL\nmissing: %s does not exist" % DRIVER)
        return 1
    failures = check(DRIVER.read_text()) + check_dt(DTSI.read_text())
    if failures:
        print("FAIL")
        for failure in failures:
            print("  " + failure)
        return 1
    print("PSTV OHCI source lifecycle contract PASS (%d invariants)"
          % (len(REQUIRED) + len(FORBIDDEN)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
