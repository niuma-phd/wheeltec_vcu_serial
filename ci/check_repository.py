#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Reject secrets, private evidence, large artifacts, and missing license tags."""

import os
from pathlib import Path
import re
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[1]
MAX_FILE_BYTES = 1024 * 1024
FORBIDDEN_SUFFIXES = {
    ".axf",
    ".bag",
    ".bin",
    ".hex",
    ".ndjson",
    ".pcap",
    ".pcapng",
    ".pdf",
    ".zip",
}
FORBIDDEN_FRAGMENTS = (
    "/ho" + "me/",
    "BENCH_" + "EVIDENCE",
    "PRIVATE_HARDWARE_" + "EVIDENCE",
)
SECRET_PATTERNS = (
    re.compile(r"-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----"),
    re.compile(r"\bghp_[A-Za-z0-9]{20,}\b"),
    re.compile(r"\bgithub_pat_[A-Za-z0-9_]{20,}\b"),
    re.compile(r"\bAKIA[0-9A-Z]{16}\b"),
)
SPDX_SUFFIXES = {".cpp", ".hpp", ".md", ".py", ".sh", ".ini", ".yml", ".yaml"}
SPDX_NAMES = {"CMakeLists.txt", "Dockerfile.ubuntu20.04"}
SPDX_EXEMPT = {"LICENSE", "NOTICE"}


def candidate_paths():
    result = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
        cwd=str(ROOT),
        check=True,
        stdout=subprocess.PIPE,
    )
    for raw in result.stdout.split(b"\0"):
        if raw:
            yield ROOT / os.fsdecode(raw)


def main():
    failures = []
    for path in candidate_paths():
        relative = path.relative_to(ROOT).as_posix()
        if path.is_symlink():
            failures.append(f"symlink is not allowed: {relative}")
            continue
        if not path.is_file():
            continue
        lower_name = path.name.lower()
        if path.suffix.lower() in FORBIDDEN_SUFFIXES or lower_name.startswith(".env"):
            failures.append(f"forbidden artifact: {relative}")
        if lower_name.endswith((".uvproj", ".uvprojx")):
            failures.append(f"forbidden firmware-project artifact: {relative}")
        size = path.stat().st_size
        if size > MAX_FILE_BYTES:
            failures.append(f"file exceeds {MAX_FILE_BYTES} bytes: {relative}")
            continue
        data = path.read_bytes()
        if b"\0" in data:
            failures.append(f"binary content is not allowed: {relative}")
            continue
        text = data.decode("utf-8", errors="strict")
        for fragment in FORBIDDEN_FRAGMENTS:
            if fragment in text:
                failures.append(f"private provenance fragment in {relative}: {fragment}")
        for pattern in SECRET_PATTERNS:
            if pattern.search(text):
                failures.append(f"possible secret in {relative}: {pattern.pattern}")
        needs_spdx = path.suffix.lower() in SPDX_SUFFIXES or path.name in SPDX_NAMES
        if needs_spdx and path.name not in SPDX_EXEMPT:
            first_lines = "\n".join(text.splitlines()[:3])
            if "SPDX-License-Identifier: Apache-2.0" not in first_lines:
                failures.append(f"missing SPDX header: {relative}")

    if failures:
        for failure in failures:
            print(f"ERROR: {failure}", file=sys.stderr)
        return 1
    print("repository content checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
