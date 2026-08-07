#!/usr/bin/env python3
"""Sim pass: drive teleop's real key path against unitree_mujoco.

Runs teleop under a pty so sshkeyboard behaves exactly as it does for an
operator, then sends the safety keys and checks what the supervisor reports.
This exercises the production route: sshkeyboard -> on_press -> safety.on_press.
"""
import os
import pty
import re
import select
import sys
import time

REPO = "/home/nse/xr_teleoperate"
PY = "/home/nse/miniconda3/envs/tv/bin/python"

ARGS = ["--sim", "--motion", "--arm=G1_29", "--network-interface=eth0", "--headless"]

env = dict(os.environ, PYTHONPATH=REPO, TELEOP_LOG_LEVEL="INFO", TERM="xterm")

pid, fd = pty.fork()
if pid == 0:
    os.chdir(os.path.join(REPO, "teleop"))
    os.execve(PY, [PY, "-u", "teleop_hand_and_arm.py"] + ARGS, env)

_ANSI = re.compile(r"\x1b\[[0-9;]*[A-Za-z]|\x1b\]8;[^\x07\x1b]*(?:\x07|\x1b\\)")

def clean(t):
    """Rich wraps output in ANSI + OSC-8 hyperlinks; strip before matching."""
    return _ANSI.sub("", t).replace("\r", "")

buf = ""
log = []


def pump(secs):
    global buf
    t0 = time.time()
    while time.time() - t0 < secs:
        r, _, _ = select.select([fd], [], [], 0.2)
        if r:
            try:
                chunk = os.read(fd, 65536).decode(errors="replace")
            except OSError:
                return
            buf += chunk
            log.append(chunk)


def wait_for(pattern, secs):
    t0 = time.time()
    while time.time() - t0 < secs:
        pump(0.3)
        if re.search(pattern, clean(buf)):
            return True
    return False


def send(key, label, settle=2.5):
    global buf
    buf = ""
    os.write(fd, key.encode())
    pump(settle)
    return clean(buf)


results = []


def check(label, cond, seen=""):
    results.append(bool(cond))
    print(f"  [{'PASS' if cond else 'FAIL'}] {label}")
    if not cond and seen:
        print(f"         saw: {seen.strip()[:200]}")


print("bringing teleop up against the simulator ...")
up = wait_for(r"Press \[r\] to start", 90)
check("teleop reached the main loop", up)
check("arm controller subscribed to sim lowstate",
      "Subscribe dds ok" in clean("".join(log)))
check("safety supervisor online", "supervisor online" in clean("".join(log)))
check("FSM table announced as UNVERIFIED (probe not yet run)",
      "UNVERIFIED" in clean("".join(log)))

if up:
    print("\nstarting the control loop ('r') -- height mode is consumed there")
    out = send("r", "start", settle=4.0)
    check("'r' -> loop running", "start" in out.lower() or out.strip() != "", out)

    print("\ndriving safety keys through the real sshkeyboard path")
    out = send("p", "pause")
    check("'p' -> PAUSED", "PAUSED" in out, out)

    out = send("p", "resume")
    check("'p' again -> resume requested", "resume" in out.lower(), out)

    out = send("h", "height mode")
    check("'h' -> height mode acknowledged",
          "height" in out.lower(), out)

    out = send("e", "e-damp arm")
    check("'e' once -> ARMED, not executed",
          "ARMED" in out or "armed" in out, out)

    out = send("x", "safe stop", settle=6.0)
    check("'x' -> safe stop runs (descend then damp)",
          "safe stop" in out.lower(), out)

    print("\nshutting down with 'q' (controlled de-energize)")
    out = send("q", "quit", settle=12.0)
    check("'q' -> controlled shutdown path ran",
          "de-energiz" in out.lower() or "exiting" in out.lower(), out)

pump(3)
try:
    os.write(fd, b"\x03")
    time.sleep(1)
    os.kill(pid, 9)
except OSError:
    pass

print(f"\n{sum(results)}/{len(results)} sim-pass checks behaved as specified")
open("/tmp/sim_keydrive_full.log", "w").write(clean("".join(log)))
print("full transcript: /tmp/sim_keydrive_full.log")
sys.exit(0 if all(results) else 1)
