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

# Empirically calibrated hybrid shot table for the hybrid shot model.
# Source data:
# - Shooter Table - Sheet1.csv first two columns with a +23.51 in hub
#   front-to-center correction on distance.
# - 10 iPhone 240 fps slow-motion videos, ignoring the first launched ball in
#   each clip.
# Restored smooth simulator-seeded shot table.
# Best-fit simulator params from tools/calibrate_shooter.py:
# slip_factor=0.448, drag_coeff=0.104, magnus_coeff=-0.4,
# launch_angle_deg=67.5.
# Each tuple is: (distance_m, flywheel_rps, time_of_flight_sec).
# These values are absolute shot targets, and load_lut_entry() converts them
# into residual corrections against the physics baseline.
_DEFAULT_CALIBRATION_ENTRY_LABELS: Final[tuple[str, str, str]] = (
    "distance_m",
    "flywheel_rps",
    "time_of_flight_sec",
)
_DEFAULT_CALIBRATION_ENTRIES: Final[tuple[tuple[float, float, float], ...]] = (
    (1.000000000000, 39.726562500000, 0.468235696748),
    (1.050000000000, 39.726562500000, 0.491737673671),
    (1.100000000000, 39.285156250000, 0.521016789639),
    (1.150000000000, 39.726562500000, 0.538760178452),
    (1.200000000000, 39.726562500000, 0.562280733858),
    (1.250000000000, 39.726562500000, 0.585807637542),
    (1.300000000000, 40.167968750000, 0.602669498863),
    (1.350000000000, 40.167968750000, 0.625952486324),
    (1.400000000000, 40.609375000000, 0.642212328061),
    (1.450000000000, 40.830078125000, 0.661675368393),
    (1.500000000000, 41.050781250000, 0.680937399102),
    (1.550000000000, 41.492187500000, 0.696293411746),
    (1.600000000000, 41.933593750000, 0.711335601461),
    (1.650000000000, 42.154296875000, 0.729859233532),
    (1.700000000000, 42.375000000000, 0.748198297971),
    (1.750000000000, 42.816406250000, 0.762422137107),
    (1.800000000000, 43.257812500000, 0.776365018260),
    (1.850000000000, 43.699218750000, 0.790035484560),
    (1.900000000000, 43.919921875000, 0.807461383062),
    (1.950000000000, 44.361328125000, 0.820636229243),
    (2.000000000000, 44.582031250000, 0.837668863814),
    (2.050000000000, 45.023437500000, 0.850371492270),
    (2.100000000000, 45.464843750000, 0.862836500603),
    (2.150000000000, 45.906250000000, 0.875070744446),
    (2.200000000000, 46.126953125000, 0.891306061640),
    (2.250000000000, 46.568359375000, 0.903113776739),
    (2.300000000000, 47.009765625000, 0.914708558675),
    (2.350000000000, 47.230468750000, 0.930404177562),
    (2.400000000000, 47.671875000000, 0.941602188684),
    (2.450000000000, 48.113281250000, 0.952603364370),
    (2.500000000000, 48.333984375000, 0.967792097993),
    (2.550000000000, 48.775390625000, 0.978423570346),
    (2.600000000000, 49.216796875000, 0.988872823894),
    (2.650000000000, 49.437500000000, 1.003584703769),
    (2.700000000000, 49.878906250000, 1.013688919448),
    (2.750000000000, 50.320312500000, 1.023624181687),
    (2.800000000000, 50.541015625000, 1.037886783872),
    (2.850000000000, 50.982421875000, 1.047499534663),
    (2.900000000000, 51.423828125000, 1.056955426919),
    (2.950000000000, 51.644531250000, 1.070794102523),
    (3.000000000000, 52.085937500000, 1.079948098745),
    (3.050000000000, 52.527343750000, 1.088956258266),
    (3.100000000000, 52.748046875000, 1.102394373774),
    (3.150000000000, 53.189453125000, 1.111119538709),
    (3.200000000000, 53.520507812500, 1.122006959791),
    (3.250000000000, 53.851562500000, 1.132768108048),
    (3.300000000000, 54.292968750000, 1.141091892347),
    (3.350000000000, 54.624023437500, 1.151600188862),
    (3.400000000000, 54.955078125000, 1.161989305760),
    (3.450000000000, 55.396484375000, 1.169936943755),
    (3.500000000000, 55.617187500000, 1.182418541605),
    (3.550000000000, 56.058593750000, 1.190126130511),
    (3.600000000000, 56.389648437500, 1.200053821571),
    (3.650000000000, 56.720703125000, 1.209872859738),
    (3.700000000000, 57.051757812500, 1.219585108288),
    (3.750000000000, 57.382812500000, 1.229192428685),
    (3.800000000000, 57.824218750000, 1.236343505670),
    (3.850000000000, 58.044921875000, 1.248099503992),
    (3.900000000000, 58.486328125000, 1.255041038340),
    (3.950000000000, 58.707031250000, 1.266608066720),
    (4.000000000000, 59.148437500000, 1.273347734085),
    (4.050000000000, 59.479492187500, 1.282358592402),
    (4.100000000000, 59.810546875000, 1.291276563679),
    (4.150000000000, 60.141601562500, 1.300103182759),
    (4.200000000000, 60.472656250000, 1.308839972493),
    (4.250000000000, 60.803710937500, 1.317488366755),
    (4.300000000000, 61.134765625000, 1.326049812975),
    (4.350000000000, 61.465820312500, 1.334525747466),
    (4.400000000000, 61.796875000000, 1.342917515467),
    (4.450000000000, 62.127929687500, 1.351226457304),
    (4.500000000000, 62.458984375000, 1.359453921324),
    (4.550000000000, 62.790039062500, 1.367601179218),
    (4.600000000000, 63.121093750000, 1.375669495499),
    (4.650000000000, 63.452148437500, 1.383660107640),
    (4.700000000000, 63.783203125000, 1.391574226778),
    (4.750000000000, 64.114257812500, 1.399413038392),
    (4.800000000000, 64.445312500000, 1.407177702954),
    (4.850000000000, 64.666015625000, 1.417273242399),
    (4.900000000000, 65.107421875000, 1.422489150005),
    (4.950000000000, 65.328125000000, 1.432443141473),
    (5.000000000000, 65.659179687500, 1.439922779823),
    (5.050000000000, 65.990234375000, 1.447333604462),
    (5.100000000000, 66.321289062500, 1.454676676429),
    (5.150000000000, 66.652343750000, 1.461952987170),
    (5.200000000000, 66.983398437500, 1.469163516638),
    (5.250000000000, 67.314453125000, 1.476309270482),
    (5.300000000000, 67.535156250000, 1.485804618857),
    (5.350000000000, 67.866210937500, 1.492823192940),
    (5.400000000000, 68.197265625000, 1.499779677568),
    (5.450000000000, 68.528320312500, 1.506674973710),
    (5.500000000000, 68.859375000000, 1.513509964576),
    (5.550000000000, 69.190429687500, 1.520285516045),
    (5.600000000000, 69.521484375000, 1.527002477074),
    (5.650000000000, 69.742187500000, 1.536078080676),
    (5.700000000000, 70.073242187500, 1.542679287191),
    (5.750000000000, 70.404296875000, 1.549224258584),
    (5.800000000000, 70.735351562500, 1.555713833451),
    (5.850000000000, 71.066406250000, 1.562148741628),
    (5.900000000000, 71.287109375000, 1.570947678912),
    (5.950000000000, 71.618164062500, 1.577274065749),
    (6.000000000000, 71.949218750000, 1.583548003788),
    (6.050000000000, 72.169921875000, 1.592191003178),
    (6.100000000000, 72.500976562500, 1.598360410776),
    (6.150000000000, 72.832031250000, 1.604479415175),
    (6.200000000000, 73.163085937500, 1.610548703139),
    (6.250000000000, 73.494140625000, 1.616568948775),
    (6.300000000000, 73.714843750000, 1.624959812650),
    (6.350000000000, 74.045898437500, 1.630881865853),
    (6.400000000000, 74.376953125000, 1.636756773195),
    (6.450000000000, 74.597656250000, 1.645005181013),
    (6.500000000000, 74.928710937500, 1.650785419469),
    (6.550000000000, 75.259765625000, 1.656520291401),
    (6.600000000000, 75.480468750000, 1.664630862505),
    (6.650000000000, 75.811523437500, 1.670274375438),
    (6.700000000000, 76.142578125000, 1.675874286640),
    (6.750000000000, 76.363281250000, 1.683851364445),
    (6.800000000000, 76.694335937500, 1.689363084267),
    (6.850000000000, 77.025390625000, 1.694832836076),
    (6.900000000000, 77.246093750000, 1.702680600113),
    (6.950000000000, 77.577148437500, 1.708065192921),
    (7.000000000000, 77.908203125000, 1.713409390869),
    (7.050000000000, 78.128906250000, 1.721131833809),
    (7.100000000000, 78.459960937500, 1.726393799051),
    (7.150000000000, 78.791015625000, 1.731616822267),
    (7.200000000000, 79.011718750000, 1.739217779796),
    (7.250000000000, 79.342773437500, 1.744361346185),
    (7.300000000000, 79.673828125000, 1.749467392207),
    (7.350000000000, 79.894531250000, 1.756950552610),
    (7.400000000000, 80.225585937500, 1.761979790213),
    (7.450000000000, 80.446289062500, 1.769387603185),
    (7.500000000000, 80.777343750000, 1.774341707803),
    (7.550000000000, 81.108398437500, 1.779260509445),
    (7.600000000000, 81.329101562500, 1.786556243191),
    (7.650000000000, 81.660156250000, 1.791402376437),
    (7.700000000000, 81.880859375000, 1.798626356218),
    (7.750000000000, 82.211914062500, 1.803401379165),
    (7.800000000000, 82.487792968750, 1.809348202847),
    (7.850000000000, 82.763671875000, 1.815260376230),
    (7.900000000000, 83.094726562500, 1.819933358967),
    (7.950000000000, 83.315429687500, 1.826982174549),
    (8.000000000000, 83.646484375000, 1.831587776250),
    (8.050000000000, 83.867187500000, 1.838569445863),
    (8.100000000000, 84.198242187500, 1.843109086922),
    (8.150000000000, 84.418945312500, 1.850024832765),
    (8.200000000000, 84.750000000000, 1.854499935123),
    (8.250000000000, 85.025878906250, 1.860147198830),
    (8.300000000000, 85.301757812500, 1.865762815001),
    (8.350000000000, 85.632812500000, 1.870145424497),
    (8.400000000000, 85.853515625000, 1.876900200785),
    (8.450000000000, 86.184570312500, 1.881221547432),
    (8.500000000000, 86.405273437500, 1.887914501225),
    (8.550000000000, 86.736328125000, 1.892175832288),
    (8.600000000000, 86.957031250000, 1.898808061665),
    (8.650000000000, 87.288085937500, 1.903010590668),
    (8.700000000000, 87.508789062500, 1.909583166026),
    (8.750000000000, 87.839843750000, 1.913728116515),
    (8.800000000000, 88.060546875000, 1.920242038723),
    (8.850000000000, 88.391601562500, 1.924330539714),
    (8.900000000000, 88.612304687500, 1.930786886415),
    (8.950000000000, 88.943359375000, 1.934820061392),
    (9.000000000000, 89.164062500000, 1.941219772406),
    (9.050000000000, 89.495117187500, 1.945198721877),
    (9.100000000000, 89.715820312500, 1.951542822040),
    (9.150000000000, 90.046875000000, 1.955468617213),
    (9.200000000000, 90.267578125000, 1.961758018484),
    (9.250000000000, 90.598632812500, 1.965631704488),
    (9.300000000000, 90.819335937500, 1.971867333921),
    (9.350000000000, 91.150390625000, 1.975689928888),
    (9.400000000000, 91.371093750000, 1.981872691172),
    (9.450000000000, 91.646972656250, 1.986836973227),
    (9.500000000000, 91.922851562500, 1.991775965173),
    (9.550000000000, 92.198730468750, 1.996689894131),
    (9.600000000000, 92.474609375000, 2.001578984399),
    (9.650000000000, 92.695312500000, 2.007634264746),
    (9.700000000000, 93.026367187500, 2.011283532230),
    (9.750000000000, 93.247070312500, 2.017288913999),
    (9.800000000000, 93.578125000000, 2.020891384381),
    (9.850000000000, 93.798828125000, 2.026847683045),
    (9.900000000000, 94.129882812500, 2.030404195087),
    (9.950000000000, 94.350585937500, 2.036312204536),
    (10.000000000000, 94.626464843750, 2.041009640355),
)


class ShotCalculator:
    """Shoot-on-the-move solver ported from the original Java implementation."""

    LOOP_PERIOD_SEC: Final[float] = 0.02
    DERIVATIVE_STEP_METERS: Final[float] = 0.01

    def __init__(self, config: Config | None = None) -> None:
        """
        Create a shot calculator and load the default calibration residuals.

        :param config: Optional solver configuration override.
        :type config: constants.shot_calculator_constants.Config | None
        """
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
        """
        Clamp a scalar value into the provided inclusive range.

        :param value: Value to clamp.
        :type value: float
        :param low: Minimum allowed value.
        :type low: float
        :param high: Maximum allowed value.
        :type high: float
        :returns: Clamped value.
        :rtype: float
        """
        return max(low, min(high, value))

    def _launcher_transform(self) -> Transform2d:
        """
        Build the launcher transform relative to the robot reference frame.
        """
        return Transform2d(
            Translation2d(self.config.launcher_offset_x, self.config.launcher_offset_y),
            Rotation2d(),
        )

    def _lookup_required(self, table: InterpolatingLookupTable, distance: float, label: str) -> float:
        """
        Read a residual lookup value or raise if the table is unexpectedly empty.

        :param table: Residual lookup table to read from.
        :type table: constants.interpolating_lookup_table.InterpolatingLookupTable
        :param distance: Shooting distance in meters used as the lookup key.
        :type distance: float
        :param label: Human-readable name of the lookup table for error reporting.
        :type label: str
        :returns: Residual value stored in the lookup table.
        :rtype: float
        """
        value = table.get(distance)
        if value is None:
            raise ValueError(f"{label} lookup table is empty")
        return value

    def _physics_rps(self, distance: float) -> float:
        """
        Get the baseline flywheel speed from the closed-form shot model.

        :param distance: Shooting distance in meters.
        :type distance: float
        :returns: Baseline flywheel target in rotations per second.
        :rtype: float
        """
        flywheel_rps, _ = calc_shot_profile(distance)
        return flywheel_rps

    def _physics_tof(self, distance: float) -> float:
        """
        Get the baseline time of flight from the closed-form shot model.

        :param distance: Shooting distance in meters.
        :type distance: float
        :returns: Baseline time of flight in seconds.
        :rtype: float
        """
        _, time_of_flight_sec = calc_shot_profile(distance)
        return time_of_flight_sec

    def load_lut_entry(self, distance_m: float, flywheel_rps: float, time_of_flight_sec: float) -> None:
        """
        Store one empirical LUT point as residuals on top of the physics model.

        :param distance_m: Shooting distance in meters represented by the LUT entry.
        :type distance_m: float
        :param flywheel_rps: Empirical flywheel target in rotations per second.
        :type flywheel_rps: float
        :param time_of_flight_sec: Empirical time of flight in seconds.
        :type time_of_flight_sec: float
        """
        # The lookup table is defined as absolute empirical shot targets:
        # distance_m -> (flywheel_rps, time_of_flight_sec).
        # Internally we store those values as residuals so the calculator uses
        # physics baseline + interpolated empirical correction.
        physics_rps = self._physics_rps(distance_m)
        physics_tof = self._physics_tof(distance_m)

        self._rps_residual_map.put(distance_m, flywheel_rps - physics_rps)
        self._tof_residual_map.put(distance_m, time_of_flight_sec - physics_tof)

    def _effective_rps(self, distance: float) -> float:
        """
        Return the calibrated flywheel target for the requested distance.

        :param distance: Shooting distance in meters.
        :type distance: float
        :returns: Calibrated flywheel target in rotations per second.
        :rtype: float
        """
        return self._physics_rps(distance) + self._lookup_required(
            self._rps_residual_map,
            distance,
            "RPS residual",
        )

    def _effective_tof(self, distance: float) -> float:
        """
        Return the calibrated time of flight for the requested distance.

        :param distance: Shooting distance in meters.
        :type distance: float
        :returns: Calibrated time of flight in seconds.
        :rtype: float
        """
        return self._physics_tof(distance) + self._lookup_required(
            self._tof_residual_map,
            distance,
            "TOF residual",
        )

    def get_profile_for_distance(self, distance: float) -> tuple[float, float]:
        """
        Get the calibrated flywheel speed and flight time for a static shot.

        :param distance: Shooting distance in meters.
        :type distance: float
        :returns: Tuple of calibrated flywheel target RPS and time of flight seconds.
        :rtype: tuple[float, float]
        """
        return self._effective_rps(distance), self._effective_tof(distance)

    def _drag_compensated_tof(self, tof: float) -> float:
        """
        Convert raw time of flight into the drag-adjusted drift time used for lead.

        :param tof: Raw time of flight in seconds.
        :type tof: float
        :returns: Drag-adjusted drift time in seconds.
        :rtype: float
        """
        if self.config.sotm_drag_coeff < 1e-6:
            return tof

        return (1.0 - math.exp(-self.config.sotm_drag_coeff * tof)) / self.config.sotm_drag_coeff

    def _tof_map_derivative(self, distance: float) -> float:
        """
        Estimate the local derivative of the calibrated TOF curve by finite difference.

        :param distance: Shooting distance in meters.
        :type distance: float
        :returns: Estimated derivative of the calibrated TOF curve.
        :rtype: float
        """
        high = self._effective_tof(distance + self.DERIVATIVE_STEP_METERS)
        low = self._effective_tof(distance - self.DERIVATIVE_STEP_METERS)
        return (high - low) / (2.0 * self.DERIVATIVE_STEP_METERS)

    def calculate(self, inputs: ShotInputs | None) -> LaunchParameters | None:
        """
        Solve a shoot-on-the-move shot using the hybrid physics-plus-LUT model.

        :param inputs: Snapshot of current drivetrain, field, and vision state.
        :type inputs: constants.shot_calculator_constants.ShotInputs | None
        :returns: Launch parameters for the calculated shot, or None when no valid solution exists.
        :rtype: constants.shot_calculator_constants.LaunchParameters | None
        """
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

        # Aim from the launcher position so drivetrain alignment matches the
        # same geometry used for shot distance and projectile compensation.
        aim_vector = compensated_target - launcher_translation
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
        """
        Collapse solver quality signals into a 0-100 shot confidence score.

        :param solver_quality: Solver convergence quality scalar from 0 to 1.
        :type solver_quality: float
        :param current_speed: Current launcher translational speed in meters per second.
        :type current_speed: float
        :param heading_error_rad: Absolute heading error to the shot angle in radians.
        :type heading_error_rad: float
        :param distance: Shooter-to-hub distance in meters.
        :type distance: float
        :param vision_confidence: External vision confidence scalar from 0 to 1.
        :type vision_confidence: float
        :returns: Confidence score from 0 to 100.
        :rtype: float
        """
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
        """
        Clear solver history so the next shot starts without warm-start state.
        """
        self._previous_tof = -1.0
        self._previous_speed = 0.0
        self._prev_robot_vx = 0.0
        self._prev_robot_vy = 0.0
        self._prev_robot_omega = 0.0
