import math
from typing import Final

from wpimath import angleModulus
from wpimath.geometry import Rotation2d, Transform2d, Translation2d, Twist2d

from constants.interpolating_lookup_table import InterpolatingLookupTable
from constants.shot_calculator_constants import (
    Config,
    LaunchParameters,
    ShotInputs,
    calc_shot_profile,
)

# ProjectileSimulator-generated seed table for the hybrid shot model.
# Each tuple is: (distance_m, flywheel_rps, time_of_flight_sec).
# These values are absolute shot targets, and load_lut_entry() converts them
# into residual corrections against the physics baseline.
_DEFAULT_CALIBRATION_ENTRY_LABELS: Final[tuple[str, str, str]] = (
    "distance_m",
    "flywheel_rps",
    "time_of_flight_sec",
)
_DEFAULT_CALIBRATION_ENTRIES: Final[tuple[tuple[float, float, float], ...]] = (
    (0.550000000000, 61.500000000000, 0.087602206052),
    (0.600000000000, 32.531250000000, 0.180790726513),
    (0.650000000000, 26.093750000000, 0.244329732490),
    (0.700000000000, 23.679687500000, 0.290161286648),
    (0.750000000000, 22.070312500000, 0.333775735358),
    (0.800000000000, 21.265625000000, 0.369770823366),
    (0.850000000000, 20.863281250000, 0.400776455753),
    (0.900000000000, 20.460937500000, 0.433005780710),
    (0.950000000000, 20.460937500000, 0.457466699570),
    (1.000000000000, 20.259765625000, 0.486678863252),
    (1.050000000000, 20.259765625000, 0.511434933508),
    (1.100000000000, 20.259765625000, 0.536220381529),
    (1.150000000000, 20.259765625000, 0.561035446842),
    (1.200000000000, 20.460937500000, 0.580215376176),
    (1.250000000000, 20.460937500000, 0.604854453921),
    (1.300000000000, 20.662109375000, 0.623503317352),
    (1.350000000000, 20.863281250000, 0.641837242141),
    (1.400000000000, 20.863281250000, 0.666109779851),
    (1.450000000000, 21.064453125000, 0.683945587800),
    (1.500000000000, 21.265625000000, 0.701489101808),
    (1.550000000000, 21.466796875000, 0.718748975381),
    (1.600000000000, 21.667968750000, 0.735733463781),
    (1.650000000000, 21.869140625000, 0.752450524222),
    (1.700000000000, 22.070312500000, 0.768907837741),
    (1.750000000000, 22.170898437500, 0.788601450260),
    (1.800000000000, 22.372070312500, 0.804599174762),
    (1.850000000000, 22.573242187500, 0.820356719877),
    (1.900000000000, 22.774414062500, 0.835880767508),
    (1.950000000000, 22.975585937500, 0.851177652865),
    (2.000000000000, 23.176757812500, 0.866253620945),
    (2.050000000000, 23.377929687500, 0.881114575337),
    (2.100000000000, 23.579101562500, 0.895766310166),
    (2.150000000000, 23.780273437500, 0.910214263866),
    (2.200000000000, 23.981445312500, 0.924463876009),
    (2.250000000000, 24.182617187500, 0.938520254094),
    (2.300000000000, 24.383789062500, 0.952388384024),
    (2.350000000000, 24.584960937500, 0.966073073420),
    (2.400000000000, 24.786132812500, 0.979579049636),
    (2.450000000000, 24.987304687500, 0.992910731401),
    (2.500000000000, 25.188476562500, 1.006072443873),
    (2.550000000000, 25.389648437500, 1.019068492312),
    (2.600000000000, 25.590820312500, 1.031902955739),
    (2.650000000000, 25.691406250000, 1.048578075976),
    (2.700000000000, 25.892578125000, 1.061117072635),
    (2.750000000000, 26.093750000000, 1.073505141533),
    (2.800000000000, 26.294921875000, 1.085745819702),
    (2.850000000000, 26.496093750000, 1.097842625632),
    (2.900000000000, 26.697265625000, 1.109798964240),
    (2.950000000000, 26.898437500000, 1.121618131159),
    (3.000000000000, 27.099609375000, 1.133303316837),
    (3.050000000000, 27.300781250000, 1.144857687404),
    (3.100000000000, 27.501953125000, 1.156284151112),
    (3.150000000000, 27.703125000000, 1.167585741218),
    (3.200000000000, 27.904296875000, 1.178765235144),
    (3.250000000000, 28.055175781250, 1.191909430797),
    (3.300000000000, 28.256347656250, 1.202856890906),
    (3.350000000000, 28.407226562500, 1.215789842769),
    (3.400000000000, 28.608398437500, 1.226514605672),
    (3.450000000000, 28.809570312500, 1.237129806251),
    (3.500000000000, 29.010742187500, 1.247637955175),
    (3.550000000000, 29.211914062500, 1.258041231003),
    (3.600000000000, 29.413085937500, 1.268342096430),
    (3.650000000000, 29.563964843750, 1.280667184827),
    (3.700000000000, 29.714843750000, 1.292906128782),
    (3.750000000000, 29.916015625000, 1.302916061235),
    (3.800000000000, 30.117187500000, 1.312831645558),
    (3.850000000000, 30.318359375000, 1.322654889759),
    (3.900000000000, 30.469238281250, 1.334535992469),
    (3.950000000000, 30.670410156250, 1.344181306532),
    (4.000000000000, 30.821289062500, 1.355897813983),
    (4.050000000000, 31.022460937500, 1.365371958380),
    (4.100000000000, 31.223632812500, 1.374762807748),
    (4.150000000000, 31.374511718750, 1.386238981927),
    (4.200000000000, 31.575683593750, 1.395468354698),
    (4.250000000000, 31.726562500000, 1.406794153258),
    (4.300000000000, 31.927734375000, 1.415868023991),
    (4.350000000000, 32.128906250000, 1.424866528849),
    (4.400000000000, 32.279785156250, 1.435972677303),
    (4.450000000000, 32.430664062500, 1.447012858445),
    (4.500000000000, 32.631835937500, 1.455792779772),
    (4.550000000000, 32.833007812500, 1.464502960857),
    (4.600000000000, 32.983886718750, 1.475338318982),
    (4.650000000000, 33.134765625000, 1.486112078944),
    (4.700000000000, 33.335937500000, 1.494618972803),
    (4.750000000000, 33.537109375000, 1.503061337243),
    (4.800000000000, 33.687988281250, 1.513643866890),
    (4.850000000000, 33.838867187500, 1.524168672009),
    (4.900000000000, 34.040039062500, 1.532421833953),
    (4.950000000000, 34.190917968750, 1.542828384813),
    (5.000000000000, 34.341796875000, 1.553179954674),
    (5.050000000000, 34.542968750000, 1.560698004922),
    (5.100000000000, 34.743896484375, 1.568204293765),
    (5.150000000000, 34.894775390625, 1.577737621565),
    (5.200000000000, 35.045654296875, 1.587203722283),
    (5.250000000000, 35.246582031250, 1.594484733961),
    (5.300000000000, 35.397460937500, 1.603773333816),
    (5.350000000000, 35.548339843750, 1.613004114339),
    (5.400000000000, 35.749267578125, 1.620075511886),
    (5.450000000000, 35.900146484375, 1.629135395102),
    (5.500000000000, 36.051025390625, 1.638136677247),
    (5.550000000000, 36.251953125000, 1.645005734709),
    (5.600000000000, 36.402832031250, 1.653844193653),
    (5.650000000000, 36.553710937500, 1.662623493754),
    (5.700000000000, 36.754638671875, 1.669299274762),
    (5.750000000000, 36.905517578125, 1.677925392599),
    (5.800000000000, 37.056396484375, 1.686491998446),
    (5.850000000000, 37.257324218750, 1.692985431630),
    (5.900000000000, 37.408203125000, 1.701410159515),
    (5.950000000000, 37.559082031250, 1.709775203664),
    (6.000000000000, 37.760009765625, 1.716098071383),
    (6.050000000000, 37.910888671875, 1.724333215395),
    (6.100000000000, 38.061767578125, 1.732508664155),
    (6.150000000000, 38.212646484375, 1.740624988055),
    (6.200000000000, 38.413574218750, 1.746732784181),
    (6.250000000000, 38.564453125000, 1.754735861440),
    (6.300000000000, 38.715332031250, 1.762679378449),
    (6.350000000000, 38.866210937500, 1.770563869798),
    (6.400000000000, 39.067138671875, 1.776473355352),
    (6.450000000000, 39.218017578125, 1.784259158245),
    (6.500000000000, 39.368896484375, 1.791984904338),
    (6.550000000000, 39.519775390625, 1.799651092506),
    (6.600000000000, 39.720703125000, 1.805375462687),
    (6.650000000000, 39.871582031250, 1.812957224936),
    (6.700000000000, 40.022460937500, 1.820478793727),
    (6.750000000000, 40.173339843750, 1.827940627464),
    (6.800000000000, 40.374267578125, 1.833495850108),
    (6.850000000000, 40.525146484375, 1.840889584687),
    (6.900000000000, 40.676025390625, 1.848222786773),
    (6.950000000000, 40.826904296875, 1.855495885040),
    (7.000000000000, 40.977783203125, 1.862709305251),
    (7.050000000000, 41.128662109375, 1.869863470898),
    (7.100000000000, 41.329589843750, 1.875143181901),
    (7.150000000000, 41.480468750000, 1.882245967937),
    (7.200000000000, 41.631347656250, 1.889288359701),
    (7.250000000000, 41.782226562500, 1.896270761740),
    (7.300000000000, 41.933105468750, 1.903193572878),
    (7.350000000000, 42.083984375000, 1.910057186724),
    (7.400000000000, 42.284912109375, 1.915075469160),
    (7.450000000000, 42.435791015625, 1.921900670218),
    (7.500000000000, 42.586669921875, 1.928665293084),
    (7.550000000000, 42.737548828125, 1.935369725680),
    (7.600000000000, 42.888427734375, 1.942014350085),
    (7.650000000000, 43.039306640625, 1.948599547171),
    (7.700000000000, 43.215209960938, 1.953366056962),
    (7.750000000000, 43.391113281250, 1.958103998722),
    (7.800000000000, 43.541992187500, 1.964668236363),
    (7.850000000000, 43.692871093750, 1.971173236808),
    (7.900000000000, 43.843750000000, 1.977619354048),
    (7.950000000000, 43.994628906250, 1.984006955429),
    (8.000000000000, 44.148925781250, 2.063663367290),
    (8.050000000000, 44.299804687500, 2.071812528641),
    (8.100000000000, 44.475830078125, 2.078791742647),
    (8.150000000000, 44.651855468750, 2.085750447052),
    (8.200000000000, 44.802734375000, 2.093835651580),
    (8.250000000000, 44.953613281250, 2.101900649349),
    (8.300000000000, 45.104492187500, 2.109945753589),
    (8.350000000000, 45.255371093750, 2.117971273629),
    (8.400000000000, 45.406250000000, 2.125977514963),
    (8.450000000000, 45.557128906250, 2.133964779327),
    (8.500000000000, 45.708007812500, 2.141933364766),
    (8.550000000000, 45.858886718750, 2.149883565705),
    (8.600000000000, 46.034912109375, 2.156664271392),
    (8.650000000000, 46.210937500000, 2.163427800228),
    (8.700000000000, 46.361816406250, 2.171323540645),
    (8.750000000000, 46.512695312500, 2.179202064735),
    (8.800000000000, 46.663574218750, 2.187063649246),
    (8.850000000000, 46.814453125000, 2.194908703674),
    (8.900000000000, 46.965332031250, 2.202737266400),
    (8.950000000000, 47.116210937500, 2.210549704654),
    (9.000000000000, 47.267089843750, 2.218346282611),
    (9.050000000000, 47.417968750000, 2.226127261450),
    (9.100000000000, 47.568847656250, 2.233893041077),
    (9.150000000000, 47.719726562500, 2.241643652343),
    (9.200000000000, 47.870605468750, 2.249379434463),
    (9.250000000000, 48.046630859375, 2.255946803293),
    (9.300000000000, 48.197509765625, 2.263653356531),
    (9.350000000000, 48.373535156250, 2.270192802818),
    (9.400000000000, 48.524414062500, 2.277871227874),
    (9.450000000000, 48.675292968750, 2.285535998811),
    (9.500000000000, 48.826171875000, 2.293187424280),
    (9.550000000000, 48.977050781250, 2.300825898941),
    (9.600000000000, 49.127929687500, 2.308451422096),
    (9.650000000000, 49.278808593750, 2.316064296963),
    (9.700000000000, 49.429687500000, 2.323664951090),
    (9.750000000000, 49.580566406250, 2.331253306396),
    (9.800000000000, 49.731445312500, 2.338829851579),
    (9.850000000000, 49.882324218750, 2.346394587948),
    (9.900000000000, 50.033203125000, 2.353947926820),
    (9.950000000000, 50.184082031250, 2.361489926974),
    (10.000000000000, 50.334960937500, 2.369020827178),
)


class ShotCalculator:
    """Shoot-on-the-move solver ported from the original Java implementation."""

    LOOP_PERIOD_SEC: Final[float] = 0.02
    DERIVATIVE_STEP_METERS: Final[float] = 0.01

    def __init__(self, config: Config | None = None) -> None:
        self.config = config if config is not None else Config()

        # Residual tables store empirical correction on top of the physics
        # baseline, indexed by shooting distance in meters.
        self._rps_residual_map = InterpolatingLookupTable()
        self._tof_residual_map = InterpolatingLookupTable()

        self._previous_tof = -1.0
        self._previous_speed = 0.0

        self._prev_robot_vx = 0.0
        self._prev_robot_vy = 0.0
        self._prev_robot_omega = 0.0

        for distance_m, flywheel_rps, time_of_flight_sec in _DEFAULT_CALIBRATION_ENTRIES:
            self.load_lut_entry(distance_m, flywheel_rps, time_of_flight_sec)

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    def _launcher_transform(self) -> Transform2d:
        return Transform2d(
            Translation2d(self.config.launcher_offset_x, self.config.launcher_offset_y),
            Rotation2d(),
        )

    def _lookup_required(self, table: InterpolatingLookupTable, distance: float, label: str) -> float:
        value = table.get(distance)
        if value is None:
            raise ValueError(f"{label} lookup table is empty")
        return value

    def _physics_rps(self, distance: float) -> float:
        flywheel_rps, _ = calc_shot_profile(distance)
        return flywheel_rps

    def _physics_tof(self, distance: float) -> float:
        _, time_of_flight_sec = calc_shot_profile(distance)
        return time_of_flight_sec

    def load_lut_entry(self, distance_m: float, flywheel_rps: float, time_of_flight_sec: float) -> None:
        # The lookup table is defined as absolute empirical shot targets:
        # distance_m -> (flywheel_rps, time_of_flight_sec).
        # Internally we store those values as residuals so the calculator uses
        # physics baseline + interpolated empirical correction.
        physics_rps = self._physics_rps(distance_m)
        physics_tof = self._physics_tof(distance_m)

        self._rps_residual_map.put(distance_m, flywheel_rps - physics_rps)
        self._tof_residual_map.put(distance_m, time_of_flight_sec - physics_tof)

    def _effective_rps(self, distance: float) -> float:
        return self._physics_rps(distance) + self._lookup_required(
            self._rps_residual_map,
            distance,
            "RPS residual",
        )

    def _effective_tof(self, distance: float) -> float:
        return self._physics_tof(distance) + self._lookup_required(
            self._tof_residual_map,
            distance,
            "TOF residual",
        )

    def _drag_compensated_tof(self, tof: float) -> float:
        if self.config.sotm_drag_coeff < 1e-6:
            return tof

        return (1.0 - math.exp(-self.config.sotm_drag_coeff * tof)) / self.config.sotm_drag_coeff

    def _tof_map_derivative(self, distance: float) -> float:
        high = self._effective_tof(distance + self.DERIVATIVE_STEP_METERS)
        low = self._effective_tof(distance - self.DERIVATIVE_STEP_METERS)
        return (high - low) / (2.0 * self.DERIVATIVE_STEP_METERS)

    def calculate(self, inputs: ShotInputs | None) -> LaunchParameters | None:
        if (
            inputs is None
            or inputs.robot_pose is None
            or inputs.field_velocity is None
            or inputs.robot_velocity is None
            or len(self._rps_residual_map) == 0
            or len(self._tof_residual_map) == 0
        ):
            return None

        raw_pose = inputs.robot_pose
        if not math.isfinite(raw_pose.x) or not math.isfinite(raw_pose.y):
            return None

        dt = self.config.phase_delay_ms / 1000.0
        robot_velocity = inputs.robot_velocity
        field_velocity = inputs.field_velocity

        ax = (robot_velocity.vx - self._prev_robot_vx) / self.LOOP_PERIOD_SEC
        ay = (robot_velocity.vy - self._prev_robot_vy) / self.LOOP_PERIOD_SEC
        angular_accel = (robot_velocity.omega - self._prev_robot_omega) / self.LOOP_PERIOD_SEC

        compensated_pose = raw_pose.exp(
            Twist2d(
                robot_velocity.vx * dt + 0.5 * ax * dt * dt,
                robot_velocity.vy * dt + 0.5 * ay * dt * dt,
                robot_velocity.omega * dt + 0.5 * angular_accel * dt * dt,
            )
        )

        self._prev_robot_vx = robot_velocity.vx
        self._prev_robot_vy = robot_velocity.vy
        self._prev_robot_omega = robot_velocity.omega

        robot_translation = compensated_pose.translation()
        heading = compensated_pose.rotation()
        to_hub_from_robot = inputs.hub_center - robot_translation

        if (
            to_hub_from_robot.x * inputs.hub_forward.x
            + to_hub_from_robot.y * inputs.hub_forward.y
        ) < 0.0:
            return None

        if (
            abs(inputs.pitch_deg) > self.config.max_tilt_deg
            or abs(inputs.roll_deg) > self.config.max_tilt_deg
        ):
            return None

        launcher_pose = compensated_pose.transformBy(self._launcher_transform())
        launcher_translation = launcher_pose.translation()
        launcher_offset_field = launcher_translation - robot_translation

        launcher_velocity = Translation2d(
            field_velocity.vx - launcher_offset_field.y * field_velocity.omega,
            field_velocity.vy + launcher_offset_field.x * field_velocity.omega,
        )

        displacement_to_hub = inputs.hub_center - launcher_translation
        distance = displacement_to_hub.norm()
        if (
            distance < self.config.min_scoring_distance
            or distance > self.config.max_scoring_distance
        ):
            return None

        robot_speed = launcher_velocity.norm()
        if robot_speed > self.config.max_sotm_speed:
            return None

        velocity_filtered = robot_speed < self.config.min_sotm_speed
        projected_distance = distance
        iterations_used = 0
        warm_start_used = False

        try:
            if velocity_filtered:
                solved_tof = self._effective_tof(distance)
            else:
                solved_tof = (
                    self._previous_tof
                    if self._previous_tof > 0.0
                    else self._effective_tof(distance)
                )
                warm_start_used = self._previous_tof > 0.0

                for iteration in range(self.config.max_iterations):
                    previous_tof = solved_tof

                    if self.config.sotm_drag_coeff < 1e-6:
                        drag_exp = 1.0
                        drift_tof = solved_tof
                    else:
                        drag_exp = math.exp(-self.config.sotm_drag_coeff * solved_tof)
                        drift_tof = (1.0 - drag_exp) / self.config.sotm_drag_coeff

                    projected_dx = displacement_to_hub.x - launcher_velocity.x * drift_tof
                    projected_dy = displacement_to_hub.y - launcher_velocity.y * drift_tof
                    projected_distance = math.hypot(projected_dx, projected_dy)

                    if projected_distance < 0.01:
                        solved_tof = self._effective_tof(distance)
                        projected_distance = distance
                        iterations_used = self.config.max_iterations + 1
                        break

                    lookup_tof = self._effective_tof(projected_distance)
                    distance_rate = -drag_exp * (
                        projected_dx * launcher_velocity.x
                        + projected_dy * launcher_velocity.y
                    ) / projected_distance
                    tof_rate = self._tof_map_derivative(projected_distance)
                    residual = lookup_tof - solved_tof
                    residual_rate = tof_rate * distance_rate - 1.0

                    if abs(residual_rate) > 0.01:
                        solved_tof -= residual / residual_rate
                    else:
                        solved_tof = lookup_tof

                    solved_tof = self.clamp(solved_tof, self.config.tof_min, self.config.tof_max)
                    iterations_used = iteration + 1

                    if abs(solved_tof - previous_tof) < self.config.convergence_tolerance:
                        break

                if not math.isfinite(solved_tof) or solved_tof < 0.0 or solved_tof > self.config.tof_max:
                    solved_tof = self._effective_tof(distance)
                    projected_distance = distance
                    iterations_used = self.config.max_iterations + 1
        except ValueError:
            return None

        self._previous_tof = solved_tof

        effective_time_of_flight = solved_tof + self.config.mech_latency_ms / 1000.0
        flywheel_rps = self._effective_rps(projected_distance)

        if velocity_filtered:
            compensated_target = inputs.hub_center
        else:
            compensated_target = inputs.hub_center - launcher_velocity * self._drag_compensated_tof(solved_tof)

        aim_vector = compensated_target - robot_translation
        drive_angle = Rotation2d(aim_vector.x, aim_vector.y)
        heading_error_rad = angleModulus(drive_angle.radians() - heading.radians())

        drive_angular_velocity = 0.0
        if not velocity_filtered and distance > 0.1:
            tangential_velocity = (
                displacement_to_hub.x * launcher_velocity.y
                - displacement_to_hub.y * launcher_velocity.x
            ) / distance
            drive_angular_velocity = tangential_velocity / distance

        if velocity_filtered:
            solver_quality = 1.0
        elif iterations_used > self.config.max_iterations:
            solver_quality = 0.0
        elif iterations_used <= 3:
            solver_quality = 1.0
        else:
            interpolation = (iterations_used - 3) / (self.config.max_iterations - 3)
            solver_quality = 1.0 + (0.1 - 1.0) * self.clamp(interpolation, 0.0, 1.0)

        confidence = self.compute_confidence(
            solver_quality=solver_quality,
            current_speed=robot_speed,
            heading_error_rad=heading_error_rad,
            distance=distance,
            vision_confidence=inputs.vision_confidence,
        )

        self._previous_speed = robot_speed

        return LaunchParameters(
            flywheel_rps=flywheel_rps,
            time_of_flight_sec=effective_time_of_flight,
            drive_angle=drive_angle,
            drive_angular_velocity_rad_per_sec=drive_angular_velocity,
            confidence=confidence,
            solved_distance_m=distance,
            iterations_used=iterations_used,
            warm_start_used=warm_start_used,
        )

    def compute_confidence(
        self,
        solver_quality: float,
        current_speed: float,
        heading_error_rad: float,
        distance: float,
        vision_confidence: float,
    ) -> float:
        speed_delta = abs(current_speed - self._previous_speed)
        velocity_stability = self.clamp(1.0 - speed_delta / 0.5, 0.0, 1.0)
        vision_quality = self.clamp(vision_confidence, 0.0, 1.0)

        distance_scale = self.clamp(
            self.config.heading_reference_distance / distance if distance > 1e-9 else 2.0,
            0.5,
            2.0,
        )
        speed_scale = 1.0 / (1.0 + self.config.heading_speed_scalar * current_speed)
        scaled_max_error = self.config.heading_max_error_rad * distance_scale * speed_scale
        if scaled_max_error <= 1e-9:
            heading_accuracy = 1.0 if abs(heading_error_rad) <= 1e-9 else 0.0
        else:
            heading_accuracy = self.clamp(
                1.0 - abs(heading_error_rad) / scaled_max_error,
                0.0,
                1.0,
            )

        range_span = self.config.max_scoring_distance - self.config.min_scoring_distance
        if range_span <= 1e-9:
            distance_in_range = 1.0
        else:
            range_fraction = (distance - self.config.min_scoring_distance) / range_span
            distance_in_range = self.clamp(
                1.0 - 2.0 * abs(range_fraction - 0.5),
                0.0,
                1.0,
            )

        components = (
            solver_quality,
            velocity_stability,
            vision_quality,
            heading_accuracy,
            distance_in_range,
        )
        weights = (
            self.config.w_convergence,
            self.config.w_velocity_stability,
            self.config.w_vision_confidence,
            self.config.w_heading_accuracy,
            self.config.w_distance_in_range,
        )

        weight_sum = 0.0
        weighted_log_sum = 0.0
        for component, weight in zip(components, weights, strict=True):
            if component <= 0.0:
                return 0.0
            weighted_log_sum += weight * math.log(component)
            weight_sum += weight

        if weight_sum <= 0.0:
            return 0.0

        return self.clamp(math.exp(weighted_log_sum / weight_sum) * 100.0, 0.0, 100.0)

    def reset_warm_start(self) -> None:
        self._previous_tof = -1.0
        self._previous_speed = 0.0
        self._prev_robot_vx = 0.0
        self._prev_robot_vy = 0.0
        self._prev_robot_omega = 0.0
