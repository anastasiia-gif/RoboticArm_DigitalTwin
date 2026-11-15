"""
Offline co-simulation harness for the DIT7 robot cell.
=====================================================
Emulates the S7-1500 scan (FB_CellController + the three modules + FB_RobotMove)
and couples it to the REAL ConveyorTransport.fmu through fmpy, plus a minimal
stand-in for the NX MCD robot (joints slew towards the commanded angles, gripper
grabs when closed at the pick pose).

Purpose: run the acceptance tests (3 consecutive cycles, controlled stop/resume,
representative fault) BEFORE NX and PLCSIM Advanced are available, and to show
the difference between SCL v0.1 and v0.2.

    python3 cell_sim.py

Timer model
-----------
S7-1500 TON records a START TIMESTAMP when IN rises; ET = now - start while IN
stays TRUE.  If the instance is simply not called for a while and is then called
again with IN still TRUE, ET includes all the wall time in between.  That is
exactly why the v0.1 grip time-out (only ever called from inside CASE 20 with
IN := TRUE) is already expired when cycle 2 reaches the grip step.
"""
import numpy as np
from fmpy import extract, read_model_description
from fmpy.fmi2 import FMU2Slave

FMU_PATH = "ConveyorTransport.fmu"
DT = 0.01                      # PLC scan / co-simulation step [s]


# --------------------------------------------------------------------------
# IEC / S7-1500 TON  (timestamp based)
# --------------------------------------------------------------------------
class TON:
    def __init__(self):
        self.start = None
        self.prev_in = False
        self.ET = 0.0
        self.Q = False

    def call(self, IN, PT, now):
        if IN:
            if not self.prev_in:
                self.start = now
            self.ET = now - self.start
            self.Q = self.ET >= PT
        else:
            self.start = None
            self.ET = 0.0
            self.Q = False
        self.prev_in = IN
        return self.Q


# --------------------------------------------------------------------------
# DB_RobotPoses
# --------------------------------------------------------------------------
# Re-solved 2026-09-20 for the wrist.  HJ_A4's axis was swung from +Z to +X,
# so A4 is no longer a second yaw (it was 0 in every pose) but the wrist
# pitch the arm was missing.  With A4 = -(A2+A3) the gripper hangs vertical
# at every taught pose and the jaws - which close along the gripper's X, the
# same axis A4 turns about - meet the block square.
#   PICK      finger centre (200, 0, 50)    tips 15 mm above the belt
#   PICK_APP  (200, 0, 130)                 80 mm straight above
#   PLACE     (40, -250, 68)                block 6 mm over the tray floor
#   PLACE_APP (40, -250, 170)
# Every pose closes to <0.3 mm at the finger centre and a swept-volume check
# of the whole cycle (joints at 90 deg/s) clears belt, stop bar, rails, tray
# rim, robot base and the unheld block.
# These MUST match DB_RobotPoses in TIA - the grip model below checks the
# robot is actually at PICK before it hands over St_Gripper_HasPart, so a
# stale copy here makes the cell fault on the grip time-out.
POSES = {
    "HOME":      (0.0,   0.0,   0.0,  0.0),
    "PICK_APP":  (0.0, -31.0, -28.6, 59.6),
    "PICK":      (0.0, -50.8, -24.4, 75.2),
    "PLACE_APP": (90.0,  -5.4, -51.3, 56.7),
    "PLACE":     (90.0, -42.1, -48.1, 90.2),
}


# --------------------------------------------------------------------------
# FB_RobotMove
# --------------------------------------------------------------------------
class RobotMove:
    def __init__(self, version):
        self.v = version
        self.prevExec = False
        self.prevStep = -1
        self.SettleT = TON()
        self.Cmd = list(POSES["HOME"])
        self.Done = False
        self.Busy = False

    def call(self, Execute, Step, T, Act, Tol, now):
        if self.v == 1:
            startMove = Execute and not self.prevExec           # rising edge ONLY
        else:
            startMove = Execute and ((not self.prevExec) or (Step != self.prevStep))

        if startMove:
            self.Busy = True
            self.Done = False

        self.prevExec = Execute
        self.prevStep = Step

        if Execute:
            self.Cmd = list(T)

        inpos = all(abs(a - t) <= Tol for a, t in zip(Act, T))

        if self.v == 1:
            self.SettleT.call(self.Busy, 0.2, now)             # never forced low
        else:
            self.SettleT.call(self.Busy and not startMove, 0.2, now)

        if self.Busy and inpos and self.SettleT.Q:
            self.Done = True
            self.Busy = False

        if not Execute:
            self.Done = False
            self.Busy = False
            if self.v == 2:
                self.prevStep = -1
        return self.Done


# --------------------------------------------------------------------------
# FB_Geven
# --------------------------------------------------------------------------
class Geven:
    def __init__(self):
        self.ArriveTO = TON()
        self.Cmd_Band_Run = False
        self.Done = False
        self.Fault = False

    def call(self, Run, SafeToRun, AtPick, now):
        if Run and SafeToRun and not self.Fault:
            if AtPick:
                self.Cmd_Band_Run = False
                self.Done = True
            else:
                self.Cmd_Band_Run = True
                self.Done = False
        else:
            self.Cmd_Band_Run = False

        self.ArriveTO.call(self.Cmd_Band_Run, 15.0, now)
        if self.ArriveTO.Q:
            self.Fault = True
            self.Cmd_Band_Run = False

        if not Run:
            self.Done = False
            self.Fault = False


# --------------------------------------------------------------------------
# FB_Verwerken
# --------------------------------------------------------------------------
class Verwerken:
    def __init__(self, version):
        self.v = version
        self.State = 0
        self.Move = RobotMove(version)
        self.GripTO = TON()
        self.Cmd = list(POSES["HOME"])
        self.Grip = False
        self.Done = False
        self.Fault = False

    def call(self, Run, SafeToRun, Act, HasPart, now):
        if self.v == 2:                                        # FIX 3: every scan
            self.GripTO.call(Run and SafeToRun and not self.Fault and self.State == 20,
                             3.0, now)

        if not Run:
            self.State = 0
            self.Done = False
            self.Fault = False
            if self.v == 2:
                self.Grip = False
            self.Move.call(False, -1, POSES["PICK_APP"], Act, 1.0, now)
        elif SafeToRun and not self.Fault:
            s = self.State
            if s == 0:
                if self.Move.call(True, 0, POSES["PICK_APP"], Act, 1.0, now):
                    self.State = 10
            elif s == 10:
                if self.Move.call(True, 10, POSES["PICK"], Act, 1.0, now):
                    self.State = 20
            elif s == 20:
                self.Grip = True
                if self.v == 1:                                # called only here
                    self.GripTO.call(True, 3.0, now)
                if HasPart:
                    self.State = 30
                elif self.GripTO.Q:
                    self.Fault = True
            elif s == 30:
                if self.Move.call(True, 30, POSES["PICK_APP"], Act, 1.0, now):
                    self.State = 100
            elif s == 100:
                self.Done = True
            self.Cmd = self.Move.Cmd


# --------------------------------------------------------------------------
# FB_Wegzetten
# --------------------------------------------------------------------------
class Wegzetten:
    def __init__(self, version):
        self.v = version
        self.State = 0
        self.Move = RobotMove(version)
        self.OpenT = TON()
        self.Cmd = list(POSES["HOME"])
        self.Grip = True
        self.Done = False
        self.Fault = False

    def call(self, Run, SafeToRun, Act, now):
        if self.v == 2:
            self.OpenT.call(Run and SafeToRun and not self.Fault and self.State == 20,
                            0.5, now)

        if not Run:
            self.State = 0
            self.Done = False
            self.Fault = False
            self.Grip = True
            self.Move.call(False, -1, POSES["PLACE_APP"], Act, 1.0, now)
        elif SafeToRun and not self.Fault:
            s = self.State
            if s == 0:
                self.Grip = True
                if self.Move.call(True, 0, POSES["PLACE_APP"], Act, 1.0, now):
                    self.State = 10
            elif s == 10:
                self.Grip = True
                if self.Move.call(True, 10, POSES["PLACE"], Act, 1.0, now):
                    self.State = 20
            elif s == 20:
                self.Grip = False
                if self.v == 1:
                    self.OpenT.call(True, 0.5, now)
                if self.OpenT.Q:
                    self.State = 30
            elif s == 30:
                self.Grip = False
                if self.Move.call(True, 30, POSES["PLACE_APP"], Act, 1.0, now):
                    self.State = 40
            elif s == 40:
                self.Grip = False
                if self.Move.call(True, 40, POSES["HOME"], Act, 1.0, now):
                    self.State = 100
            elif s == 100:
                self.Grip = False
                self.Done = True
            self.Cmd = self.Move.Cmd


# --------------------------------------------------------------------------
# FB_CellController
# --------------------------------------------------------------------------
class CellController:
    def __init__(self, version, cycles=3):
        self.v = version
        self.State = 0
        self.Hold = False
        self.prevStop = False
        self.LoadReq = False
        self.LoadPulse = TON()
        self.Cycles = cycles
        self.Geven = Geven()
        self.Verwerken = Verwerken(version)
        self.Wegzetten = Wegzetten(version)
        self.Cmd_A = list(POSES["HOME"])
        self.Cmd_Grip = False
        self.Cmd_Band = False
        self.Cmd_LoadNew = False
        self.Cmd_Picked = False
        self.Sts_Fault = False
        self.Sts_Done = False
        self.Sts_CycleCount = 0

    def scan(self, Start, Stop, Reset, Auto, Safe, AtPick, HasPart, Act, now):
        if Stop and not self.prevStop:
            self.Hold = True
        self.prevStop = Stop

        if self.v == 2:
            if (not Safe) or (not Auto):
                self.Hold = True
            if Start and Safe and Auto:
                self.Hold = False
        else:
            if Start:
                self.Hold = False

        gAct = self.State == 10
        pAct = self.State == 20
        wAct = self.State == 30

        self.Geven.call(gAct and not self.Hold, Safe, AtPick, now)
        self.Verwerken.call(pAct, Safe and not self.Hold, Act, HasPart, now)
        self.Wegzetten.call(wAct, Safe and not self.Hold, Act, now)

        s = self.State
        if s == 0:
            self.Cmd_Band = False
            self.Cmd_Grip = False
            self.Cmd_Picked = False
            self.Cmd_A = list(POSES["HOME"])
            self.Sts_CycleCount = 0
            self.Sts_Done = False
            self.Sts_Fault = False
            if self.v == 1:
                self.Cmd_LoadNew = False
            if Start and Auto and Safe:
                self.LoadReq = True
                if self.v == 1:
                    self.Cmd_LoadNew = True
                self.State = 10
        elif s == 10:
            if self.v == 1:
                self.Cmd_LoadNew = False
            self.Cmd_Band = self.Geven.Cmd_Band_Run
            if self.Geven.Fault:
                self.State = 100
            if self.Geven.Done:
                self.State = 20
        elif s == 20:
            self.Cmd_Band = False
            self.Cmd_A = list(self.Verwerken.Cmd)
            self.Cmd_Grip = self.Verwerken.Grip
            if HasPart:
                self.Cmd_Picked = True
            if self.Verwerken.Fault:
                self.State = 100
            if self.Verwerken.Done:
                self.State = 30
        elif s == 30:
            self.Cmd_A = list(self.Wegzetten.Cmd)
            self.Cmd_Grip = self.Wegzetten.Grip
            if self.Wegzetten.Fault:
                self.State = 100
            if self.Wegzetten.Done:
                self.State = 40
        elif s == 40:
            self.Sts_CycleCount += 1
            self.Cmd_Picked = False
            if self.Sts_CycleCount < self.Cycles:
                self.LoadReq = True
                if self.v == 1:
                    self.Cmd_LoadNew = True
                self.State = 10
            else:
                self.State = 50
        elif s == 50:
            self.Sts_Done = True
            self.Cmd_Band = False
            if Reset:
                self.State = 0
        elif s == 100:
            self.Sts_Fault = True
            self.Cmd_Band = False
            if Reset:
                self.Sts_Fault = False
                self.State = 0

        if self.v == 2:
            self.LoadPulse.call(self.LoadReq, 0.2, now)
            if self.LoadPulse.Q:
                self.LoadReq = False
            self.Cmd_LoadNew = self.LoadReq

        if self.Hold and 10 <= self.State <= 30:
            self.Cmd_Band = False
            self.Cmd_A = list(Act)


# --------------------------------------------------------------------------
# Plant: NX MCD stand-in (robot + gripper) + the real FMU (belt + product)
# --------------------------------------------------------------------------
class Plant:
    def __init__(self, unzipdir, md, slew=45.0, allow_grip=True):
        self.slew = slew                     # joint speed [deg/s]
        self.allow_grip = allow_grip         # set False to force a grip fault
        self.Act = list(POSES["HOME"])
        self.HasPart = False
        self.vr = {v.name: v.valueReference for v in md.modelVariables}
        self.fmu = FMU2Slave(guid=md.guid, unzipDirectory=unzipdir,
                             modelIdentifier=md.coSimulation.modelIdentifier,
                             instanceName="belt")
        self.fmu.instantiate()
        self.fmu.setupExperiment(startTime=0.0)
        self.fmu.enterInitializationMode()
        self.fmu.exitInitializationMode()
        self.AtPick = False
        self.ProductPos = 0.0
        # set by the bridge from the NX presence sensor CS_Pick; None = use the FMU
        self.sensor_at_pick = None

    def step(self, Cmd_A, Cmd_Grip, Cmd_Band, Cmd_LoadNew, Cmd_Picked, t, dt):
        # --- robot joints slew towards the set-points (stand-in for NX) ---
        step = self.slew * dt
        self.Act = [a + max(-step, min(step, c - a)) for a, c in zip(self.Act, Cmd_A)]

        # --- gripper: grabs when closed at the PICK pose with a product present ---
        at_pick_pose = all(abs(a - t_) <= 1.0 for a, t_ in zip(self.Act, POSES["PICK"]))
        if Cmd_Grip and at_pick_pose and self.AtPick and self.allow_grip:
            self.HasPart = True
        if not Cmd_Grip:
            self.HasPart = False

        # --- FMU: belt + product transport + light barrier ---
        self.fmu.setBoolean([self.vr['BeltRun']], [Cmd_Band])
        self.fmu.setBoolean([self.vr['Picked']], [Cmd_Picked])
        self.fmu.setBoolean([self.vr['LoadNew']], [Cmd_LoadNew])
        self.fmu.doStep(currentCommunicationPoint=t, communicationStepSize=dt)
        self.AtPick = self.fmu.getBoolean([self.vr['ProductAtPick']])[0] != 0
        if self.sensor_at_pick is not None:
            self.AtPick = bool(self.sensor_at_pick)
        self.ProductPos = self.fmu.getReal([self.vr['ProductPos']])[0]


# --------------------------------------------------------------------------
# Test runner
# --------------------------------------------------------------------------
def run(version, tmax=120.0, stop_at=None, resume_after=2.0,
        allow_grip=True, cycles=3, label=""):
    unzipdir = extract(FMU_PATH)
    md = read_model_description(FMU_PATH)
    plant = Plant(unzipdir, md, allow_grip=allow_grip)
    cell = CellController(version, cycles=cycles)

    t = 0.0
    log = []
    stopped_at = None
    prev_count = 0
    events = []

    while t < tmax:
        Start = (t < 0.05)
        Stop = False
        Reset = False
        if stop_at is not None and stop_at <= t < stop_at + 0.05:
            Stop = True
        if stop_at is not None and stop_at + resume_after <= t < stop_at + resume_after + 0.05:
            Start = True
        if cell.Sts_Fault and t > 0.1:
            if stopped_at is None:
                stopped_at = t
                events.append(("FAULT", round(t, 2)))
            if t > stopped_at + 1.0:
                Reset = True

        cell.scan(Start, Stop, Reset, True, True,
                  plant.AtPick, plant.HasPart, plant.Act, t)
        plant.step(cell.Cmd_A, cell.Cmd_Grip, cell.Cmd_Band,
                   cell.Cmd_LoadNew, cell.Cmd_Picked, t, DT)

        if cell.Sts_CycleCount != prev_count:
            events.append((f"cycle {cell.Sts_CycleCount} complete", round(t, 2)))
            prev_count = cell.Sts_CycleCount
        log.append((t, cell.State, plant.ProductPos, cell.Sts_CycleCount))
        if cell.Sts_Done:
            events.append(("ALL CYCLES DONE", round(t, 2)))
            break
        t += DT

    plant.fmu.terminate()
    plant.fmu.freeInstance()

    print(f"\n=== {label} (SCL v0.{version}) ===")
    print(f"  cycles completed : {cell.Sts_CycleCount} / {cycles}")
    print(f"  final state      : {cell.State}   Done={cell.Sts_Done}  Fault={cell.Sts_Fault}")
    print(f"  events           : {events}")
    return cell, np.array(log)


if __name__ == "__main__":
    print("Offline co-simulation of the robot cell against ConveyorTransport.fmu")

    run(1, label="TEST A  3 consecutive cycles")
    run(2, label="TEST A  3 consecutive cycles")

    run(2, stop_at=4.0, resume_after=3.0, label="TEST B  controlled stop + resume")

    run(2, allow_grip=False, tmax=40.0, label="TEST C  representative fault (gripper never grips)")
