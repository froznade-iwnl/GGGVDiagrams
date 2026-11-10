"""
Pure-python port of the C++ `VehicleModelValidation` (OpenCarDynamics / TUM)
that was originally shipped in this repository as a colcon/ros2 built package.

Why this exists:
    The original toolchain required building several C++ packages with colcon
    and sourcing a ROS2-style install setup before the python scripts could
    import them. That workflow is Ubuntu-only and awkward on macOS (Apple
    Silicon). This module re-implements the *exact same* validation vehicle
    model (point-mass model with kinematic steering relation, force-saturated
    to a friction circle of a_max = 20 m/s^2 that is moved down by 2 m/s^2 to
    emulate drag forces) so the whole g-g-g-v toolchain runs on pure python
    on any platform, including macOS arm64.

Public API mirrors the pybind11 bindings used previously:
    VehicleModelValidation()
        .step()
        .set_driver_input(DriverInput)
        .set_external_influences(ExternalInfluences)
        .get_output() -> VehicleModelOutput
        .reset()
        .get_param_manager() -> ParamManager
            .get_parameter_value(name) -> Parameter (.as_double())
            .set_parameter_value(name, value)
"""

from dataclasses import dataclass, field
import math


# ---------------------------------------------------------------------------
# Types (equivalent of tum_sim_types_py / tum_types_py)
# ---------------------------------------------------------------------------
@dataclass
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0


@dataclass
class TorquesPerAxle:
    front_left: float = 0.0
    front_right: float = 0.0
    rear_left: float = 0.0
    rear_right: float = 0.0


@dataclass
class DriverInput:
    steering_angle_rad: float = 0.0
    wheel_torque_Nm: TorquesPerAxle = field(default_factory=TorquesPerAxle)


@dataclass
class ExternalInfluences:
    external_force_N: Vec3 = field(default_factory=Vec3)


@dataclass
class VehicleModelOutput:
    position_m: Vec3 = field(default_factory=Vec3)
    orientation_rad: Vec3 = field(default_factory=Vec3)
    velocity_mps: Vec3 = field(default_factory=Vec3)
    angular_velocity_radps: Vec3 = field(default_factory=Vec3)
    acceleration_mps2: Vec3 = field(default_factory=Vec3)
    steering_angle: float = 0.0


class _Parameter:
    """Mimics the param_manager handle returned by get_parameter_value()."""

    __slots__ = ("_value",)

    def __init__(self, value):
        self._value = float(value)

    def as_double(self):
        return self._value


class ParamManager:
    """Minimal port of tam::core::ParamManager."""

    def __init__(self):
        self._params = {}

    def declare_parameter(self, name, default):
        if name not in self._params:
            self._params[name] = float(default)

    def get_parameter_value(self, name):
        return _Parameter(self._params[name])

    def set_parameter_value(self, name, value):
        # mirror the C++ behavior: unknown names are silently ignored
        if name in self._params:
            self._params[name] = float(value)


# ---------------------------------------------------------------------------
# State vector indices (StateVectorValidationModel in C++)
# ---------------------------------------------------------------------------
_X_M = 0
_Y_M = 1
_PSI_RAD = 2
_V_MPS = 3


class _IMR:
    def __init__(self):
        self.Fx_request = 0.0
        self.Fy_request = 0.0
        self.F_request = 0.0
        self.F_actual = 0.0
        self.Fx = 0.0
        self.Fy = 0.0
        self.ay = 0.0
        self.yaw_rate = 0.0


def _clamp(v, lo, hi):
    return min(max(v, lo), hi)


def _rk4_step(f, t, x, h):
    """Fixed-step classical RK4 integrator.

    Replaces the Dormand-Prince 4(5) integrator of the C++ model; for the
    smooth dynamics of this point-mass model both integrate the ODE with
    negligible difference while RK4 is deterministic and cheap.
    """
    k1 = f(t, x)
    k2 = f(t + h / 2.0, [x[i] + h / 2.0 * k1[i] for i in range(len(x))])
    k3 = f(t + h / 2.0, [x[i] + h / 2.0 * k2[i] for i in range(len(x))])
    k4 = f(t + h, [x[i] + h * k3[i] for i in range(len(x))])
    return [x[i] + h / 6.0 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
            for i in range(len(x))]


class VehicleModelValidation:
    """Point-mass validation vehicle model (pure python port)."""

    A_MAX_MPS2 = 20.0     # friction circle radius (hardcoded in C++ model)
    F_DRAG_N_PER_M = 2.0  # drag force = 2 * m_vehicle (moved circle down)

    def __init__(self):
        self.param_manager = ParamManager()
        self.driver_input = DriverInput()
        self.external_influences = ExternalInfluences()
        self.model_output = VehicleModelOutput()
        self.imr = _IMR()
        self.x = [0.0, 0.0, 0.0, 0.0]
        self._declare_parameters()
        self.reset()

    # -- parameter handling --------------------------------------------------
    def _declare_parameters(self):
        pm = self.param_manager
        pm.declare_parameter("integration_step_size_s", 0.001)
        pm.declare_parameter("vehicle_parameters.dist_cg_rear_m", 1.5)
        pm.declare_parameter("vehicle_parameters.dist_cg_front_m", 1.5)
        pm.declare_parameter("initial_state.x_m", 0.0)
        pm.declare_parameter("initial_state.y_m", 0.0)
        pm.declare_parameter("initial_state.psi_rad", 0.0)
        pm.declare_parameter("initial_state.vehicle_dynamics.v_x_mps", 20.0)
        pm.declare_parameter("vehicle_dynamics_double_track.m", 782.0)
        pm.declare_parameter("vehicle_dynamics_double_track.l_f", 1.5)
        pm.declare_parameter("vehicle_dynamics_double_track.l", 3.0)
        pm.declare_parameter("vehicle_dynamics_double_track.b_r", 1.5)
        pm.declare_parameter("vehicle_dynamics_double_track.rr_w_f", 0.3)
        pm.declare_parameter("vehicle_dynamics_double_track.rr_w_r", 0.3)
        pm.declare_parameter("steering_actuator.static_offset_rad", 0.0)
        pm.declare_parameter("initial_state.drivetrain.omega_FL_radps", 0.0)
        pm.declare_parameter("initial_state.drivetrain.omega_FR_radps", 0.0)
        pm.declare_parameter("initial_state.drivetrain.omega_RL_radps", 0.0)
        pm.declare_parameter("initial_state.drivetrain.omega_RR_radps", 0.0)

    def get_param_manager(self):
        return self.param_manager

    # -- interfaces ----------------------------------------------------------
    def set_driver_input(self, driver_input):
        self.driver_input = driver_input

    def set_external_influences(self, external_influences):
        self.external_influences = external_influences

    def get_output(self):
        return self.model_output

    def reset(self):
        pm = self.param_manager
        self.x[_X_M] = pm.get_parameter_value("initial_state.x_m").as_double()
        self.x[_Y_M] = pm.get_parameter_value("initial_state.y_m").as_double()
        self.x[_PSI_RAD] = pm.get_parameter_value("initial_state.psi_rad").as_double()
        self.x[_V_MPS] = pm.get_parameter_value("initial_state.vehicle_dynamics.v_x_mps").as_double()

    # -- dynamics ------------------------------------------------------------
    def _ode(self, t, x):
        pm = self.param_manager
        l_r = pm.get_parameter_value("vehicle_parameters.dist_cg_rear_m").as_double()
        l_f = pm.get_parameter_value("vehicle_parameters.dist_cg_front_m").as_double()
        r_tire = 0.5 * pm.get_parameter_value("vehicle_dynamics_double_track.rr_w_r").as_double() \
               + 0.5 * pm.get_parameter_value("vehicle_dynamics_double_track.rr_w_f").as_double()
        m_vehicle = pm.get_parameter_value("vehicle_dynamics_double_track.m").as_double()

        a_max = self.A_MAX_MPS2
        f_drag = self.F_DRAG_N_PER_M * m_vehicle

        imr = self.imr
        x_dot = [0.0, 0.0, 0.0, 0.0]
        x_dot[_X_M] = x[_V_MPS] * math.cos(x[_PSI_RAD])
        x_dot[_Y_M] = x[_V_MPS] * math.sin(x[_PSI_RAD])

        di = self.driver_input.wheel_torque_Nm
        imr.Fx_request = (di.front_left + di.front_right + di.rear_left + di.rear_right) / r_tire

        curvature_driven = math.tan(self.driver_input.steering_angle_rad) / (l_f + l_r)
        imr.Fy_request = m_vehicle * curvature_driven * x[_V_MPS] ** 2

        imr.F_request = math.hypot(imr.Fx_request, imr.Fy_request)
        imr.F_actual = _clamp(imr.F_request, 0.0, a_max * m_vehicle)

        f_max = a_max * m_vehicle
        imr.Fx = _clamp(imr.Fx_request, -f_max, f_max)
        fy_max = math.sqrt(max(f_max ** 2 - imr.Fx ** 2, 0.0))
        imr.Fy = _clamp(imr.Fy_request, -fy_max, fy_max)

        imr.ay = imr.Fy / m_vehicle
        if x[_V_MPS] < 1.0:
            imr.yaw_rate = x[_V_MPS] * math.tan(self.driver_input.steering_angle_rad) / (l_r + l_f)
        else:
            imr.yaw_rate = imr.ay / x[_V_MPS]

        x_dot[_PSI_RAD] = imr.yaw_rate
        x_dot[_V_MPS] = (imr.Fx - f_drag
                         + self.external_influences.external_force_N.x) / m_vehicle
        return x_dot

    def step(self):
        h = self.param_manager.get_parameter_value("integration_step_size_s").as_double()

        # evaluate ODE at current state (used for outputs, like in the C++ code)
        x_dot = self._ode(0.0, self.x)

        # integrate one fixed step
        self.x = _rk4_step(self._ode, 0.0, self.x, h)

        out = self.model_output
        out.position_m.x = self.x[_X_M]
        out.position_m.y = self.x[_Y_M]
        out.orientation_rad.z = self.x[_PSI_RAD]
        out.velocity_mps.x = self.x[_V_MPS]
        out.velocity_mps.y = 0.0
        out.angular_velocity_radps.z = x_dot[_PSI_RAD]
        out.acceleration_mps2.x = x_dot[_V_MPS]
        out.acceleration_mps2.y = self.imr.ay
        out.steering_angle = self.driver_input.steering_angle_rad
