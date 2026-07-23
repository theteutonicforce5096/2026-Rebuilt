from typing import Any

from phoenix6.status_code import StatusCode
from wpilib import Timer, reportWarning, wait


def configure_device(
    device: Any,
    configs: Any,
    num_attempts: int,
    *,
    per_attempt_timeout: float = 0.25,
    retry_delay: float = 0.1,
    max_wait: float = 5.0,
) -> None:
    """
    Configures a CTRE device with the specified configs once it is on the bus.

    Waits for the device to enumerate on its CAN bus before applying configs, then
    retries until the device confirms OK or the time budget runs out. Gating on each
    device's own connection covers both the CANivore and rio buses without special-casing,
    so each device is configured as soon as it is ready rather than after a fixed delay.

    :param device: The CTRE device (motor controller, CANcoder, CANdle) to configure.
    :type device: Any
    :param configs: The configuration to apply to the device.
    :type configs: Any
    :param num_attempts: Number of times to attempt to configure the device.
    :type num_attempts: int
    :param per_attempt_timeout: Seconds each apply call waits for the device to respond.
    :type per_attempt_timeout: float
    :param retry_delay: Seconds to wait between attempts while the device comes up.
    :type retry_delay: float
    :param max_wait: Total seconds to spend waiting for the device before giving up.
    :type max_wait: float
    """
    deadline = Timer.getFPGATimestamp() + max_wait

    # Devices on the CANivore ("Drivetrain") can take a few seconds to enumerate after
    # the RIO boots, so wait for the device to answer before configuring it.
    while not device.is_connected and Timer.getFPGATimestamp() < deadline:
        wait(retry_delay)

    # Apply configs, retrying until the device confirms OK or we run out of time. The
    # attempt count still caps retries once the bus is up so a bad config fails fast.
    status_code: StatusCode = StatusCode.STATUS_CODE_NOT_INITIALIZED
    for _ in range(num_attempts):
        status_code = device.configurator.apply(configs, per_attempt_timeout)
        if status_code.is_ok():
            return
        if Timer.getFPGATimestamp() >= deadline:
            break
        wait(retry_delay)

    reportWarning(
        f"Device with CAN ID {device.device_id} failed to config with error: {status_code.name}",
        False,
    )
