"""
plcsim_opcua_bridge.py - PLCSIM Advanced  <->  OPC UA  <->  NX MCD
==================================================================
Runs on the VINCI laptop, beside PLCSIM Advanced.  Replaces the internal plant
model of plcsim_bridge.py with the real NX model on the other laptop:

    PLCSIM Advanced ──Simulation Runtime API──> this script
                                                     │  OPC UA (dials OUT)
                                                     ▼
                            opcua_server.py  [VictusVectus 192.168.2.58:4840]
                                                     ▲
                                     MCD OPC UA client, via 127.0.0.1

Every connection leaves this laptop, which is the only direction its endpoint
security permits.

    python plcsim_opcua_bridge.py --check
    python plcsim_opcua_bridge.py --run
    python plcsim_opcua_bridge.py --run --stop-at 4.0      # stop/resume test
    python plcsim_opcua_bridge.py --run --fmu              # FMU owns the belt

Reads and writes are BATCHED: all nine commands go out in one OPC UA request
and all six statuses come back in one more, so a step costs about two round
trips (~14 ms measured) rather than fifteen (~100 ms).

Gripper: the PLC emits a boolean Cmd_Gripper_Close; MCD wants a finger
position.  This maps it to 0 mm open / 15 mm closed and publishes both, so the
PLC interface stays boolean while the model gets a travel it can animate.
"""
BRIDGE_VERSION = "2026-09-21-opcua-c  (NX presence sensor + plant joints)"

import argparse
import asyncio
import glob
import os
import sys
import time

# ----------------------------------------------------------------- PLCSIM API
DLL_GLOBS = [
    r"C:\Program Files (x86)\Common Files\Siemens\PLCSIMADV\API\*\Siemens.Simatic.Simulation.Runtime.Api.x64.dll",
    r"C:\Program Files\Common Files\Siemens\PLCSIMADV\API\*\Siemens.Simatic.Simulation.Runtime.Api.x64.dll",
    r"C:\Program Files (x86)\Siemens\Automation\PLCSIMADV\API\*\Siemens.Simatic.Simulation.Runtime.Api.x64.dll",
]

# PLC -> NX
TAGS_FROM_PLC = {
    "Cmd_Band_Run": "bool", "Cmd_Gripper_Close": "bool",
    "Cmd_LoadNew": "bool", "Cmd_Picked": "bool",
    "Cmd_A1_Pos": "real", "Cmd_A2_Pos": "real",
    "Cmd_A3_Pos": "real", "Cmd_A4_Pos": "real",
}
# PLC diagnostics, read but not published
TAGS_STATUS = {
    "Sts_State": "int", "Sts_CycleCount": "int",
    "Sts_Fault": "bool", "Sts_Done": "bool", "Sts_Hold": "bool",
}
# NX -> PLC
TAGS_TO_PLC = {
    "St_ProductAtPick": "bool", "St_Gripper_HasPart": "bool",
    "St_A1_Pos": "real", "St_A2_Pos": "real",
    "St_A3_Pos": "real", "St_A4_Pos": "real",
}
OPERATOR = ["Cmd_Start", "Cmd_Stop", "Cmd_Reset", "Cmd_Auto", "SafeToRun"]

# jaw travel: gap = 28 - 2*travel, so -16 -> 60 mm (clears the 50 mm
# product) and -10.5 -> 49 mm (1 mm interference, the grip)
FINGER_OPEN, FINGER_CLOSED = -16.0, -10.5


def find_dll():
    for pattern in DLL_GLOBS:
        hits = sorted(glob.glob(pattern))
        if hits:
            return hits[-1]
    return None


def load_api(verbose=True):
    dll = find_dll()
    if not dll:
        sys.exit("Could not find the PLCSIM Advanced API DLL. Add its folder "
                 "to DLL_GLOBS at the top of this script.")
    if verbose:
        print(f"BRIDGE  : {BRIDGE_VERSION}")
        print(f"API DLL : {dll}")
    try:
        import clr
    except ImportError:
        sys.exit("pythonnet is not installed.  Run:  pip install pythonnet")
    clr.AddReference(dll)
    from Siemens.Simatic.Simulation.Runtime import SimulationRuntimeManager
    return SimulationRuntimeManager


def get_instance(mgr, name=None, verbose=True):
    infos = list(mgr.RegisteredInstanceInfo)
    if not infos:
        sys.exit("No PLCSIM Advanced instances are registered. Start one, "
                 "download the TIA project, and put the CPU in RUN.")
    if verbose:
        print("Instances:", ", ".join(i.Name for i in infos))
    chosen = name or infos[0].Name
    if verbose:
        print(f"Using   : {chosen}")
    return mgr.CreateInterface(chosen)


def _try(obj, names, *args):
    for n in names:
        f = getattr(obj, n, None)
        if f is not None:
            return f(*args)
    raise AttributeError(f"none of {names} exist on the instance object")


def read(inst, tag, kind):
    if kind == "bool":
        return bool(_try(inst, ["ReadBool"], tag))
    if kind == "real":
        return float(_try(inst, ["ReadFloat", "ReadReal", "ReadSingle"], tag))
    return int(_try(inst, ["ReadInt16", "ReadInt"], tag))


def write(inst, tag, kind, value):
    if kind == "bool":
        return _try(inst, ["WriteBool"], tag, bool(value))
    if kind == "real":
        from System import Single
        return _try(inst, ["WriteFloat", "WriteReal", "WriteSingle"], tag,
                    Single(float(value)))
    from System import Int16
    return _try(inst, ["WriteInt16", "WriteInt"], tag, Int16(int(value)))


def ensure_running(inst, verbose=True):
    state = str(getattr(inst, "OperatingState", "?"))
    if verbose:
        print(f"State   : {state}")
    if state.lower() == "run":
        return True
    for call in ("PowerOn", "Run"):
        f = getattr(inst, call, None)
        if f:
            try:
                f()
                time.sleep(1.0)
            except Exception as e:                            # noqa: BLE001
                if verbose:
                    print(f"   {call}() -> {e}")
    return str(getattr(inst, "OperatingState", "?")).lower() == "run"


def update_tags(inst, verbose=True):
    try:
        from Siemens.Simatic.Simulation.Runtime import ETagListDetails
        for name in ("IOMDB", "IOM", "IO"):
            e = getattr(ETagListDetails, name, None)
            if e is None:
                continue
            try:
                inst.UpdateTagList(e, True)
                break
            except Exception:                                 # noqa: BLE001
                continue
    except Exception:                                         # noqa: BLE001
        try:
            inst.UpdateTagList()
        except Exception:                                     # noqa: BLE001
            pass
    tags = list(getattr(inst, "TagInfos", []) or [])
    if verbose:
        print(f"Taglist : {len(tags)} tags visible")
    return tags


# --------------------------------------------------------------------- OPC UA
async def resolve_nodes(client):
    """Find Objects/RobotCell and return {name: node}."""
    folder = None
    for child in await client.nodes.objects.get_children():
        if (await child.read_browse_name()).Name == "RobotCell":
            folder = child
            break
    if folder is None:
        raise RuntimeError("no 'RobotCell' folder on the OPC UA server - is "
                           "opcua_server.py the one running?")
    nodes = {}
    for var in await folder.get_children():
        nodes[(await var.read_browse_name()).Name] = var
    return nodes


async def check(url, instance_name):
    from asyncua import Client
    mgr = load_api()
    inst = get_instance(mgr, instance_name)
    ensure_running(inst)
    tags = update_tags(inst)
    names = {getattr(t, "Name", str(t)) for t in tags}

    print("\nPLC tags:")
    missing = [t for t in list(TAGS_FROM_PLC) + list(TAGS_TO_PLC) +
               list(TAGS_STATUS) + OPERATOR if t not in names]
    print("   all present" if not missing else f"   MISSING: {missing}")

    print(f"\nOPC UA {url}")
    async with Client(url=url, timeout=10) as client:
        nodes = await resolve_nodes(client)
        print(f"   {len(nodes)} variables")
        need = list(TAGS_FROM_PLC) + list(TAGS_TO_PLC) + ["Cmd_FingerPos"]
        gone = [n for n in need if n not in nodes]
        print("   all present" if not gone else f"   MISSING: {gone}")
    print("\nReady to --run" if not missing and not gone else
          "\nFix the missing names above first.")


async def run(url, instance_name, tmax, stop_at, resume_after, use_plant, nx_sensor=True):
    from asyncua import Client
    mgr = load_api()
    inst = get_instance(mgr, instance_name)
    if not ensure_running(inst):
        print("\nWarning: the CPU is not in RUN; nothing will move.")
    if not update_tags(inst):
        sys.exit("No program in the instance - download the TIA project first.")

    plant = None
    if use_plant:
        import glob as _glob
        here = os.path.dirname(os.path.abspath(__file__))
        for d in (here, os.path.join(here, "FMU"), os.getcwd(),
                  os.path.join(os.getcwd(), "FMU")):
            if os.path.exists(os.path.join(d, "cell_sim.py")):
                sys.path.insert(0, d)
                break
        from fmpy import extract, read_model_description
        import cell_sim as cs
        fmu = None
        for d in (here, os.path.join(here, "FMU"), os.getcwd(),
                  os.path.join(os.getcwd(), "FMU")):
            c = os.path.join(d, "ConveyorTransport.fmu")
            if os.path.exists(c):
                fmu = c
                break
        if fmu is None:
            sys.exit("ConveyorTransport.fmu not found - put the FMU folder "
                     "next to this script, or run with --no-plant")
        cs.FMU_PATH = fmu
        plant = cs.Plant(extract(fmu), read_model_description(fmu))
        print(f"Plant   : {fmu}")
        print("          joints slewed at %.0f deg/s to match the MCD position controls" % plant.slew)
        print("          product at pick: %s" % ("NX presence sensor CS_Pick" if nx_sensor
                                                 else "FMU light barrier"))
    else:
        print("Plant   : none - St_* taken from NX (needs sensors wired in MCD)")

    cmd_names = list(TAGS_FROM_PLC) + ["Cmd_FingerPos"]
    st_names = list(TAGS_TO_PLC)

    async with Client(url=url, timeout=10) as client:
        nodes = await resolve_nodes(client)
        cmd_nodes = [nodes[n] for n in cmd_names]
        st_nodes = [nodes[n] for n in st_names]
        print(f"OPC UA  : {url}  ({len(nodes)} variables)")

        # operator: Auto + safety on, everything else off
        for t in OPERATOR:
            write(inst, t, "bool", False)
        write(inst, "Cmd_Auto", "bool", True)
        write(inst, "SafeToRun", "bool", True)
        time.sleep(0.3)

        st = read(inst, "Sts_State", "int")
        if st != 0:
            print(f"Reset   : cell was in state {st}, sending Cmd_Reset")
            for _ in range(20):
                write(inst, "Cmd_Reset", "bool", True)
                time.sleep(0.15)
                write(inst, "Cmd_Reset", "bool", False)
                time.sleep(0.15)
                st = read(inst, "Sts_State", "int")
                if st == 0:
                    break
        if st != 0:
            sys.exit(f"Cell will not leave state {st}. Check Sts_Fault.")
        print("Reset   : cell at state 0 (IDLE)\n")

        print("  t[s]  state  cyc  band grip   Cmd_A1..A4")
        t_sim, started = 0.0, False
        t0 = t_prev = time.perf_counter()
        prev_state = prev_count = None
        events, lat = [], []

        while t_sim < tmax:
            now = time.perf_counter()
            dt = min(now - t_prev, 0.1)
            t_prev = now

            cmd = {t: read(inst, t, k) for t, k in TAGS_FROM_PLC.items()}
            sts = {t: read(inst, t, k) for t, k in TAGS_STATUS.items()}

            values = [cmd[n] for n in TAGS_FROM_PLC]
            values.append(FINGER_CLOSED if cmd["Cmd_Gripper_Close"]
                          else FINGER_OPEN)

            tw = time.perf_counter()
            await client.write_values(cmd_nodes, values)
            st_vals = await client.read_values(st_nodes)
            lat.append((time.perf_counter() - tw) * 1000.0)

            back = dict(zip(st_names, st_vals))
            if plant is not None:
                # The NX model animates from Cmd_* but publishes nothing back,
                # so the plant supplies every status the controller waits on:
                # the light barrier, the grip check and the joint positions.
                if nx_sensor:
                    plant.sensor_at_pick = bool(back["St_ProductAtPick"])
                plant.step([cmd[f"Cmd_A{i}_Pos"] for i in range(1, 5)],
                           cmd["Cmd_Gripper_Close"], cmd["Cmd_Band_Run"],
                           cmd["Cmd_LoadNew"], cmd["Cmd_Picked"], t_sim, dt)
                back["St_ProductAtPick"] = plant.AtPick
                back["St_Gripper_HasPart"] = plant.HasPart
                for i in range(1, 5):
                    back[f"St_A{i}_Pos"] = plant.Act[i - 1]
            for t, k in TAGS_TO_PLC.items():
                write(inst, t, k, back[t])

            if not started and t_sim > 0.5:
                write(inst, "Cmd_Start", "bool", True)
                time.sleep(0.15)
                write(inst, "Cmd_Start", "bool", False)
                started = True
                events.append(("Start", round(t_sim, 2)))
            if stop_at and stop_at <= t_sim < stop_at + 0.05:
                write(inst, "Cmd_Stop", "bool", True); time.sleep(0.15)
                write(inst, "Cmd_Stop", "bool", False)
                events.append(("Stop", round(t_sim, 2)))
            if stop_at and stop_at + resume_after <= t_sim < stop_at + resume_after + 0.05:
                write(inst, "Cmd_Start", "bool", True); time.sleep(0.15)
                write(inst, "Cmd_Start", "bool", False)
                events.append(("Resume", round(t_sim, 2)))

            if sts["Sts_State"] != prev_state:
                print(f"  {t_sim:5.2f}  {sts['Sts_State']:5d}  "
                      f"{sts['Sts_CycleCount']:3d}   "
                      f"{int(cmd['Cmd_Band_Run'])}    "
                      f"{int(cmd['Cmd_Gripper_Close'])}   "
                      f"{cmd['Cmd_A1_Pos']:7.1f}{cmd['Cmd_A2_Pos']:7.1f}"
                      f"{cmd['Cmd_A3_Pos']:7.1f}{cmd['Cmd_A4_Pos']:7.1f}",
                      flush=True)
                prev_state = sts["Sts_State"]
            if sts["Sts_CycleCount"] != prev_count:
                if prev_count is not None:
                    events.append((f"cycle {sts['Sts_CycleCount']}",
                                   round(t_sim, 2)))
                prev_count = sts["Sts_CycleCount"]
            if sts["Sts_Fault"]:
                events.append(("FAULT", round(t_sim, 2)))
                print("\nPLC reports FAULT")
                break
            if sts["Sts_Done"]:
                events.append(("ALL CYCLES DONE", round(t_sim, 2)))
                break

            t_sim += dt
            await asyncio.sleep(0.005)

    print("\nEvents:", events)
    if lat:
        lat.sort()
        print(f"OPC UA round trip: median {lat[len(lat)//2]:.1f} ms, "
              f"max {lat[-1]:.1f} ms over {len(lat)} steps")
    print("This ran your compiled SCL on the real PLC against the NX model.")


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--url", default="opc.tcp://192.168.2.58:4840/robotcell/")
    p.add_argument("--instance", default=None)
    p.add_argument("--check", action="store_true")
    p.add_argument("--run", action="store_true")
    p.add_argument("--tmax", type=float, default=600.0)
    p.add_argument("--stop-at", type=float, default=None)
    p.add_argument("--resume-after", type=float, default=3.0)
    p.add_argument("--no-plant", action="store_true",
                   help="take St_* from NX instead of the FMU/joint model")
    p.add_argument("--fmu-presence", action="store_true",
                   help="use the FMU light barrier instead of the NX sensor CS_Pick")
    a = p.parse_args()

    if a.check:
        asyncio.run(check(a.url, a.instance))
    elif a.run:
        asyncio.run(run(a.url, a.instance, a.tmax, a.stop_at,
                        a.resume_after, not a.no_plant, not a.fmu_presence))
    else:
        p.print_help()
