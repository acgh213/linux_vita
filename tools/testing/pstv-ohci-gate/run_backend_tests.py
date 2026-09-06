#!/usr/bin/env python3
"""Compile the real PSTV backend against host-only API doubles and run it."""
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[3]
HERE = Path(__file__).resolve().parent
HEADERS = "capability completion debugfs delay dma-mapping interrupt io ioport irqdomain module mutex of_irq of_platform platform_device pm_runtime seq_file slab suspend uaccess usb usb/hcd types"

def main():
    cc = shutil.which("cc") or shutil.which("gcc")
    if not cc:
        print("compiler not found", file=sys.stderr); return 2
    with tempfile.TemporaryDirectory(prefix="pstv-ohci-gate-host-") as td:
        shadow = Path(td); (shadow / "linux").mkdir()
        for name in HEADERS.split():
            target = shadow / "linux" / f"{name}.h"
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_text("/* host shadow header */\n")
        binary = Path(td) / "test_backend"
        cmd = [cc, "-std=gnu11", "-Wall", "-Wextra", "-Werror",
               "-Wno-unused-function", "-Wno-unused-parameter", "-g",
               "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-I" + str(shadow),
               str(HERE / "test_backend.c"),
               str(ROOT / "drivers/usb/host/pstv-ohci-gate-core.c"),
               "-o", str(binary)]
        build = subprocess.run(cmd, cwd=ROOT, text=True, capture_output=True)
        sys.stdout.write(build.stdout)
        sys.stderr.write(build.stderr)
        if build.returncode:
            return build.returncode
        run = subprocess.run([str(binary)], cwd=ROOT, text=True, capture_output=True)
        sys.stdout.write(run.stdout); sys.stderr.write(run.stderr)
        return run.returncode

if __name__ == "__main__":
    raise SystemExit(main())
