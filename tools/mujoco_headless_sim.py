#!/usr/bin/env python3
"""unitree_mujoco without the viewer (copy into unitree_mujoco/simulate_python/).

The stock unitree_mujoco.py gates its whole simulation loop on
`while viewer.is_running():`, and `mujoco.viewer.launch_passive` hangs under
WSLg — so nothing is ever published. Model loading and physics are fine
headless (verified: 200 steps in 0.03 s), and the DDS bridge does not need a
window, so this runs the same bridge and physics without one.

Everything else is stock: same config.py, same UnitreeSdk2Bridge, same
domain/interface.

    python headless_sim.py [seconds]      # default: run until Ctrl-C
"""
import sys
import time

import mujoco
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

import config
from unitree_sdk2py_bridge import UnitreeSdk2Bridge

RUN_S = float(sys.argv[1]) if len(sys.argv) > 1 else float("inf")

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)
mj_model.opt.timestep = config.SIMULATE_DT
print(f"[headless] {config.ROBOT}: nq={mj_model.nq} nu={mj_model.nu} "
      f"dt={mj_model.opt.timestep}", flush=True)

ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
bridge = UnitreeSdk2Bridge(mj_model, mj_data)
print(f"[headless] DDS domain {config.DOMAIN_ID} on {config.INTERFACE}; "
      f"idl={'unitree_hg' if bridge.idl_type else 'unitree_go'}", flush=True)

# Settle the robot onto the ground before publishing motion, so the first
# lowstate a consumer sees is a plausible pose rather than a spawn transient.
for _ in range(int(1.0 / mj_model.opt.timestep)):
    mujoco.mj_step(mj_model, mj_data)
print("[headless] settled; publishing lowstate. Ctrl-C to stop.", flush=True)

t_end = time.time() + RUN_S
n = 0
try:
    while time.time() < t_end:
        t0 = time.perf_counter()
        mujoco.mj_step(mj_model, mj_data)
        n += 1
        if n % 2000 == 0:
            print(f"[headless] {n} steps, sim t={mj_data.time:.1f}s", flush=True)
        slack = mj_model.opt.timestep - (time.perf_counter() - t0)
        if slack > 0:
            time.sleep(slack)
except KeyboardInterrupt:
    pass
print(f"[headless] stopped after {n} steps ({mj_data.time:.1f}s sim time)", flush=True)
