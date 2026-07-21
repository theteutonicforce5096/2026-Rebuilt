from typing import Any

from wpilib import DriverStation
from phoenix6.status_code import StatusCode

def configure_device(device: Any, configs: Any, num_attempts: int) -> None:
    """
    Configures a CTRE device with the specified configs, retrying up to num_attempts times.

    :param device: The CTRE device (motor controller, CANcoder, CANdle) to configure.
    :type device: Any
    :param configs: The configuration to apply to the device.
    :type configs: Any
    :param num_attempts: Number of times to attempt to configure the device.
    :type num_attempts: int
    """
    # Seed with a non-OK sentinel so the failure path is well defined even when
    # num_attempts <= 0 and the loop body never runs.
    status_code: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
    for _ in range(num_attempts):
        status_code = device.configurator.apply(configs)
        if status_code.is_ok():
            break
    if not status_code.is_ok():
        DriverStation.reportWarning(
            f"Device with CAN ID {device.device_id} failed to config with error: {status_code.name}",
            False,
        )
