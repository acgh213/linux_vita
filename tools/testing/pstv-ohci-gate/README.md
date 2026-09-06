# PSTV OHCI diagnostic — WORK IN PROGRESS, NOT DEPLOYABLE

This isolated candidate is based on the protected B16 baseline at commit
`ed91f5ce13d1`; the current committed source tip. It is a post-boot
laboratory diagnostic, not an OHCI host driver and not a direct-keyboard
implementation.

`CONFIG_USB_PSTV_OHCI_GATE` is disabled by default and built-in only.
Initialization registers a privileged debugfs interface; it must not touch
OHCI at boot. The triggers are `read`, `reset`, and `frame` under
`/sys/kernel/debug/pstv-ohci-gate/trigger`, with the last record in `result`.
The external PSTV Type-A port must remain empty for this controller-only test.

## Current validation boundary

- Host core, actual-backend, lifecycle, and 86-case fault/ownership harnesses
  pass with `-Wall -Wextra -Werror` and AddressSanitizer/UndefinedBehaviorSanitizer.
- ARM `W=1` object compilation passes for both production objects.
- Full ARM `W=1 zImage` build passes from the clean source tip; the image has
  no `-dirty` suffix. The PSTV DTB was built with the canonical Vita
  preprocess/dtc pipeline and parses successfully. Three existing DTS-schema
  warnings from `vita.dtsi` are recorded in the lab evidence.
- Checkpatch reports 0 errors and 0 warnings for all three production files.
- These tests and builds do not prove actual OHCI read/reset/frame/IRQ/DMA
  behavior. The earlier diagnostic candidate's hardware gate passed, but this
  HCD-wiring revision has not been deployed. Independent review and a new
  hardware enumeration gate remain pending.

## Host sequencing/transaction harness

From the kernel root:

```sh
cc -std=c11 -Wall -Wextra -Werror -fsanitize=address,undefined \
  -fno-omit-frame-pointer tools/testing/pstv-ohci-gate/test_lifecycle.c \
  drivers/usb/host/pstv-ohci-gate-core.c -o /tmp/pstv-ohci-host-tests
/tmp/pstv-ohci-host-tests
```

The lifecycle harness also invokes the core harness. It covers normal
read/reset/frame transactions, acquisition failure, stuck HCR and failed
quiescence retention; this is not an exhaustive backend failure-path suite.

## Evidence and continuation

`/home/cassie/projects/vita-linux-research/lab/usb-re/ohci-gate-2026-09-06/`
contains the plan, protected rollback manifest, live baseline observations,
`host-tests.json`, and `BLOCKER.md` with the exact refused command and log.
Resolve the execution blocker with the operator; do not bypass the guard.
Then complete backend failure-path validation, ARM build/checkpatch and
independent review before freezing a clean full-build candidate. Preserve
the B16 embedded rootfs, existing loader and PSTV DTB byte-identically.
Hardware stages remain separate with stop-on-failure and baseline recovery.
