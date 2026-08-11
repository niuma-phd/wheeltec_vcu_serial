<!-- SPDX-License-Identifier: Apache-2.0 -->
# Security policy

Please report security or safety-sensitive defects privately through GitHub's
security-advisory interface rather than a public issue. Include the affected
revision, platform, reproduction using a fake transport or PTY, and the observed
fail-closed behavior.

Never attach vehicle captures, device identifiers, credentials, firmware, or
manufacturer source to a report. Redact paths and use synthetic frames.

No release should weaken a limit, watchdog, authorization gate, zero retry, or
reconnect interlock merely to make a test pass.

The integer authorization and emergency-stop reset tokens prevent stale values
from being replayed within a process. They are deliberately not secrets or
identity credentials. Protect the CLI input channel and any wrapper API with
deployment-level process, network, and middleware access controls.
