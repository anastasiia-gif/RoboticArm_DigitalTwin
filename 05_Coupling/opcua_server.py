"""
opcua_server.py - the meeting point between NX MCD and PLCSIM Advanced.
=======================================================================
Runs on the NX laptop (VictusVectus).  Both sides connect to it as clients,
which is what makes this work at all:

    plcsim_bridge.py on the VINCI laptop   ->  dials OUT to 192.168.2.58:4840
    MCD's OPC UA client inside NX          ->  dials out to 127.0.0.1:4840

VINCI's endpoint security silently drops inbound TCP, so nothing may connect
INTO that laptop; every connection here leaves it.  MCD's connection never
leaves this machine at all.

    python opcua_server.py                 # serve on all interfaces, port 4840
    python opcua_server.py --port 4841     # if 4840 is taken
    python opcua_server.py --watch         # also print values as they change

Endpoint for the NX dialog:   opc.tcp://127.0.0.1:4840/robotcell/
Endpoint for the bridge:      opc.tcp://192.168.2.58:4840/robotcell/

Anonymous, no encryption - a lab co-simulation on a private LAN.  Say so in
the report rather than pretending otherwise; a production twin would use
certificates and a signed endpoint.
"""
import argparse
import asyncio
import logging

from asyncua import Server, ua

NS = "http://fontys.dit7/robotcell"

# name, type, who writes it.  Direction is from the PLC's point of view, the
# same convention as the TIA tag table and NX_HANDOVER.md.
SIGNALS = [
    # PLC writes, NX consumes
    ("Cmd_Band_Run",       bool,  "PLC->NX"),
    ("Cmd_Gripper_Close",  bool,  "PLC->NX"),
    ("Cmd_LoadNew",        bool,  "PLC->NX"),
    ("Cmd_Picked",         bool,  "PLC->NX"),
    ("Cmd_A1_Pos",         float, "PLC->NX"),
    ("Cmd_A2_Pos",         float, "PLC->NX"),
    ("Cmd_A3_Pos",         float, "PLC->NX"),
    ("Cmd_A4_Pos",         float, "PLC->NX"),
    ("Cmd_FingerPos",      float, "PLC->NX"),
    # NX writes, PLC consumes
    ("St_ProductAtPick",   bool,  "NX->PLC"),
    ("St_Gripper_HasPart", bool,  "NX->PLC"),
    ("St_A1_Pos",          float, "NX->PLC"),
    ("St_A2_Pos",          float, "NX->PLC"),
    ("St_A3_Pos",          float, "NX->PLC"),
    ("St_A4_Pos",          float, "NX->PLC"),
]


async def main(port, host, watch):
    server = Server()
    await server.init()
    endpoint = f"opc.tcp://{host}:{port}/robotcell/"
    server.set_endpoint(endpoint)
    server.set_server_name("DIT7 RobotCell Digital Twin")
    server.set_security_policy([ua.SecurityPolicyType.NoSecurity])

    idx = await server.register_namespace(NS)
    folder = await server.nodes.objects.add_object(idx, "RobotCell")

    nodes = {}
    for name, kind, direction in SIGNALS:
        initial = False if kind is bool else 0.0
        vtype = ua.VariantType.Boolean if kind is bool else ua.VariantType.Double
        var = await folder.add_variable(idx, name, initial, vtype)
        await var.set_writable()          # both clients write; MCD needs this
        nodes[name] = var

    print("=" * 68)
    print("  DIT7 RobotCell OPC UA server")
    print("=" * 68)
    print(f"  listening on   : {endpoint}")
    print(f"  namespace      : {NS}  (index {idx})")
    print(f"  variables      : {len(nodes)} under Objects/RobotCell")
    print()
    print("  In NX  : External Signal Configuration -> OPC UA tab")
    print(f"           Endpoint URL   opc.tcp://127.0.0.1:{port}/robotcell/")
    print("  On VINCI:")
    print(f"           opc.tcp://192.168.2.58:{port}/robotcell/")
    print()
    print("  Ctrl-C to stop.")
    print("=" * 68, flush=True)

    last = {}
    async with server:
        while True:
            await asyncio.sleep(0.2)
            if not watch:
                continue
            for name, var in nodes.items():
                v = await var.read_value()
                if last.get(name) != v:
                    last[name] = v
                    shown = v if isinstance(v, bool) else round(v, 2)
                    print(f"   {name:20s} = {shown}", flush=True)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=4840)
    p.add_argument("--host", default="0.0.0.0",
                   help="0.0.0.0 serves both loopback and the LAN")
    p.add_argument("--watch", action="store_true",
                   help="print every value change")
    p.add_argument("--debug", action="store_true")
    a = p.parse_args()
    if a.debug:
        logging.basicConfig(level=logging.INFO)
    try:
        asyncio.run(main(a.port, a.host, a.watch))
    except KeyboardInterrupt:
        print("\nstopped.")
