# DIT7 Digital Twin – Resit Assignment 1: Robot Pick-&-Place with Conveyor Belt
Nastia Polimasheva – Fontys, Digital Twin (resit)

Siemens NX 2312 MCD  ↔  OPC UA  ↔  TIA Portal / S7-1500 on PLCSIM Advanced V7.0, plus an FMI 2.0 FMU.

## Contents

| Folder | What it contains |
|---|---|
| `01_Report` | The technical report (.docx and .pdf). |
| `02_PLC_TIA` | TIA Portal V19 project archive `Robotic_arm.zap19.zip` (unzip, then Project → Retrieve). The SCL sources are included for reading: `RobotCell_Assignment1_v3.scl`, `DB_RobotPoses.scl`, `FB_Verwerken.txt`, `FB_CellController.txt`. The TIA project is the authoritative version. |
| `03_NX_MCD` | `RobotCell_v2.prt` (the MCD model) and its component parts. Open `RobotCell_v2.prt` in NX 2312 → Mechatronics Concept Designer. |
| `04_FMU` | `ConveyorTransport.fmu` (FMI 2.0 co-simulation), its Python source, the verification script and the verification / convergence figures (report Chapter 9). |
| `05_Coupling` | `opcua_server.py` (OPC UA server, 15 variables), `plcsim_opcua_bridge.py` (PLCSIM Advanced ↔ OPC UA bridge) and `FMU/cell_sim.py` (joint/grip plant model and the offline co-simulation harness). |
| `07_Evidence` | `final_3cycles.mp4` / `.txt`: three closed-loop cycles in NX under the PLC. `final_stop_resume_and_fault.txt`: Stop/resume, then the arrival-watchdog FAULT. `final_run_2.txt`: a second three-cycle run. `earlier_runs/`: earlier hardware-in-the-loop runs with the FMU. |

## How to reproduce a run

1. **NX machine:** `python 05_Coupling/opcua_server.py`. It serves `opc.tcp://0.0.0.0:4840/robotcell/`.
2. Open `03_NX_MCD/RobotCell_v2.prt` → Automation → External Signal Configuration → OPC UA tab. Check the endpoint `opc.tcp://127.0.0.1:4840/robotcell/` shows Connected → OK → **Play**.
3. **PLC machine:** download the TIA project to a PLCSIM Advanced instance `Robot_Cell` and set it to RUN.
4. `python 05_Coupling/plcsim_opcua_bridge.py --run`. Use `--url opc.tcp://<NX-machine-IP>:4840/robotcell/` if the IP is not the default 192.168.2.58.
   - Stop/resume test: add `--stop-at 12 --resume-after 4`.
   - `--fmu-presence` uses the FMU light barrier instead of the NX presence sensor.

## Key points of the final model

- **Presence sensor:** `CS_Pick` at the stop bar is published as `St_ProductAtPick`. It starts every pick and confirms it: after the lift the sensor must be clear.
- **Pick retry:** in `FB_Verwerken` (FIX 6), a missed pick is retried twice before FAULT.
- **Feed:** `OS_Feed` drops one block when `Cmd_LoadNew && !csPick && !csSpawn && !csBelt`. That means only on the PLC's request at the end of a cycle, and only when the belt is empty.
- **Gripper:** `GR_Grip` is a native MCD Vacuum gripper. It holds the block while `Cmd_FingerPos > -13` and releases when `< -13`.
- **Poses:** HOME 0/0/0/0, PICK_APP 0/-31.0/-28.6/59.6, PICK 0/-50.8/-24.4/75.2, PLACE_APP 90/-5.4/-51.3/56.7, PLACE 90/-42.1/-48.1/90.2. They must be identical in `DB_RobotPoses` and `cell_sim.py`.
