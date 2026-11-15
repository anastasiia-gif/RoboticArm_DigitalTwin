"""
ConveyorTransport FMU
=====================
Co-simulation FMU (FMI 2.0) modelling the "Give" stage of the pick-&-place cell:
a conveyor belt driven by a motor that transports a rigid product towards the
robot pick position, where a light barrier detects it.

Physics
-------
1. Belt motor speed - first-order lag towards a commanded set-point:
       d(v)/dt = (v_target - v) / tau         with v_target = v_max if BeltRun else 0
   Discrete (explicit Euler, step h):
       v[k+1] = v[k] + h * (v_target - v[k]) / tau

2. Product transport - kinematic integration of belt speed (no slip):
       d(x)/dt = v        ->   x[k+1] = x[k] + h * v[k]
   The product stops advancing once it has been picked (Picked = true) so the
   sensor signal stays stable for the PLC handshake.

3. Light barrier at the pick position:
       ProductAtPick = (x >= x_sensor)

Interface (matches the PLC tag table)
-------------------------------------
  IN  BeltRun        Bool   <- Cmd_Band_Run      (PLC output)
  IN  Picked         Bool   <- St handshake when robot has gripped the part
  IN  LoadNew        Bool   <- pulse to drop a fresh product at x_start (new cycle)
  OUT BeltSpeed      Real   -> diagnostic / HMI
  OUT ProductPos     Real   -> diagnostic / NX visual
  OUT ProductAtPick  Bool   -> St_ProductAtPick   (PLC input, light barrier)
  PAR v_max          Real   max belt speed [m/s]
  PAR tau            Real   motor time constant [s]
  PAR x_sensor       Real   pick / light-barrier position [m]
  PAR x_start        Real   product spawn position [m]
"""
from pythonfmu import Fmi2Causality, Fmi2Slave, Fmi2Variability, Boolean, Real


class ConveyorTransport(Fmi2Slave):

    author = "DIT7 Resit - Assignment 1"
    description = "First-order conveyor motor + product transport + light barrier"

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # --- parameters ---
        self.v_max = 0.20      # m/s
        self.tau = 0.30        # s
        self.x_sensor = 0.50   # m
        self.x_start = 0.00    # m

        # --- states / outputs ---
        self.BeltSpeed = 0.0
        self.ProductPos = self.x_start
        self.ProductAtPick = False

        # --- inputs ---
        self.BeltRun = False
        self.Picked = False
        self.LoadNew = False

        self._prev_loadnew = False

        # ---- register variables ----
        self.register_variable(Real("v_max", causality=Fmi2Causality.parameter,
                                    variability=Fmi2Variability.tunable))
        self.register_variable(Real("tau", causality=Fmi2Causality.parameter,
                                    variability=Fmi2Variability.tunable))
        self.register_variable(Real("x_sensor", causality=Fmi2Causality.parameter,
                                    variability=Fmi2Variability.tunable))
        self.register_variable(Real("x_start", causality=Fmi2Causality.parameter,
                                    variability=Fmi2Variability.tunable))

        self.register_variable(Boolean("BeltRun", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))
        self.register_variable(Boolean("Picked", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))
        self.register_variable(Boolean("LoadNew", causality=Fmi2Causality.input, variability=Fmi2Variability.discrete))

        self.register_variable(Real("BeltSpeed", causality=Fmi2Causality.output))
        self.register_variable(Real("ProductPos", causality=Fmi2Causality.output))
        self.register_variable(Boolean("ProductAtPick", causality=Fmi2Causality.output, variability=Fmi2Variability.discrete))

    def do_step(self, current_time, step_size):
        h = step_size

        # rising-edge detection on LoadNew -> respawn product at start
        if self.LoadNew and not self._prev_loadnew:
            self.ProductPos = self.x_start
            self.ProductAtPick = False
        self._prev_loadnew = self.LoadNew

        # 1) motor first-order speed response
        v_target = self.v_max if self.BeltRun else 0.0
        if self.tau > 1e-6:
            self.BeltSpeed += h * (v_target - self.BeltSpeed) / self.tau
        else:
            self.BeltSpeed = v_target

        # 2) product transport (frozen once picked)
        if not self.Picked:
            self.ProductPos += h * self.BeltSpeed

        # 3) light barrier
        self.ProductAtPick = self.ProductPos >= self.x_sensor

        return True
