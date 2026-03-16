from __future__ import annotations

"""
RobotPy/WPIMath port of ShotCalculator.java.

This preserves the Java-facing API shape as closely as practical in Python:
- module path: frc.firecontrol.ShotCalculator
- class name: ShotCalculator
- nested types: ShotCalculator.LaunchParameters, ShotCalculator.ShotInputs, ShotCalculator.Config
- camelCase public methods retained for easier drop-in migration

Note: RobotPy exposes the geometry/kinematics primitives used here, but this file includes
small local shims for WPILib Java's MathUtil.clamp/interpolate behavior and
InterpolatingDoubleTreeMap semantics.
"""

from bisect import bisect_left
from dataclasses import dataclass
import math
from typing import Optional

from wpimath import angleModulus
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Twist2d
from wpimath.kinematics import ChassisSpeeds


class _MathUtil:
    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))

    @staticmethod
    def interpolate(start: float, end: float, t: float) -> float:
        t = _MathUtil.clamp(t, 0.0, 1.0)
        return start + (end - start) * t


class _InterpolatingDoubleTreeMap:
    """
    Minimal Python equivalent of WPILib Java's InterpolatingDoubleTreeMap.

    Behavior implemented to match typical WPILib usage in this solver:
    - exact key -> exact value
    - below minimum key -> minimum value
    - above maximum key -> maximum value
    - between keys -> linear interpolation
    """

    def __init__(self) -> None:
        self._keys: list[float] = []
        self._values: list[float] = []

    def put(self, key: float, value: float) -> None:
        idx = bisect_left(self._keys, key)
        if idx < len(self._keys) and self._keys[idx] == key:
            self._values[idx] = value
        else:
            self._keys.insert(idx, key)
            self._values.insert(idx, value)

    def get(self, key: float) -> Optional[float]:
        if not self._keys:
            return None

        idx = bisect_left(self._keys, key)

        if idx < len(self._keys) and self._keys[idx] == key:
            return self._values[idx]
        if idx == 0:
            return self._values[0]
        if idx >= len(self._keys):
            return self._values[-1]

        x0 = self._keys[idx - 1]
        x1 = self._keys[idx]
        y0 = self._values[idx - 1]
        y1 = self._values[idx]

        if x1 == x0:
            return y0

        t = (key - x0) / (x1 - x0)
        return y0 + (y1 - y0) * t

    def clear(self) -> None:
        self._keys.clear()
        self._values.clear()


class ShotCalculator:
    @dataclass(frozen=True)
    class LaunchParameters:
        rpm: float
        timeOfFlightSec: float
        driveAngle: Rotation2d
        driveAngularVelocityRadPerSec: float
        isValid: bool
        confidence: float
        solvedDistanceM: float
        iterationsUsed: int
        warmStartUsed: bool

    @dataclass(frozen=True)
    class ShotInputs:
        robotPose: Pose2d
        fieldVelocity: ChassisSpeeds
        robotVelocity: ChassisSpeeds
        hubCenter: Translation2d
        hubForward: Translation2d
        visionConfidence: float
        pitchDeg: float = 0.0
        rollDeg: float = 0.0

    class Config:
        def __init__(self) -> None:
            # Launcher geometry (measure from CAD)
            self.launcherOffsetX = 0.20
            self.launcherOffsetY = 0.0

            # How close/far you can score from (meters)
            self.minScoringDistance = 0.5
            self.maxScoringDistance = 5.0

            # Newton solver tuning
            self.maxIterations = 25
            self.convergenceTolerance = 0.001
            self.tofMin = 0.05
            self.tofMax = 5.0

            # Below this speed (m/s), don't bother with SOTM, just aim straight
            self.minSOTMSpeed = 0.1

            # Above this speed (m/s), don't shoot, we're outside calibration range
            self.maxSOTMSpeed = 3.0

            # Latency compensation (ms)
            self.phaseDelayMs = 30.0
            self.mechLatencyMs = 20.0

            # The ball's inherited robot velocity decays in flight because of drag.
            self.sotmDragCoeff = 0.47

            # Confidence scoring weights (5-component weighted geometric mean)
            self.wConvergence = 1.0
            self.wVelocityStability = 0.8
            self.wVisionConfidence = 1.2
            self.wHeadingAccuracy = 1.5
            self.wDistanceInRange = 0.5
            self.headingMaxErrorRad = math.radians(15.0)

            # Heading tolerance tuning
            self.headingSpeedScalar = 1.0
            self.headingReferenceDistance = 2.5

            # Suppress firing when pitch or roll exceeds this threshold
            self.maxTiltDeg = 5.0

    DERIV_H = 0.01

    def __init__(self, config: Optional[Config] = None) -> None:
        self.config = config if config is not None else ShotCalculator.Config()

        self.rpmMap = _InterpolatingDoubleTreeMap()
        self.tofMap = _InterpolatingDoubleTreeMap()
        self.correctionRpmMap = _InterpolatingDoubleTreeMap()
        self.correctionTofMap = _InterpolatingDoubleTreeMap()

        self.rpmOffset = 0.0

        self.previousTOF = -1.0
        self.previousSpeed = 0.0

        self.prevRobotVx = 0.0
        self.prevRobotVy = 0.0
        self.prevRobotOmega = 0.0

    def loadLUTEntry(self, distanceM: float, rpm: float, tof: float) -> None:
        self.rpmMap.put(distanceM, rpm)
        self.tofMap.put(distanceM, tof)

    def effectiveRPM(self, distance: float) -> float:
        base = self.rpmMap.get(distance)
        if base is None:
            raise ValueError("RPM lookup table is empty")
        correction = self.correctionRpmMap.get(distance)
        return base + (correction if correction is not None else 0.0) + self.rpmOffset

    def effectiveTOF(self, distance: float) -> float:
        base = self.tofMap.get(distance)
        if base is None:
            raise ValueError("TOF lookup table is empty")
        correction = self.correctionTofMap.get(distance)
        return base + (correction if correction is not None else 0.0)

    def dragCompensatedTOF(self, tof: float) -> float:
        c = self.config.sotmDragCoeff
        if c < 1e-6:
            return tof
        return (1.0 - math.exp(-c * tof)) / c

    def tofMapDerivative(self, d: float) -> float:
        t_high = self.effectiveTOF(d + self.DERIV_H)
        t_low = self.effectiveTOF(d - self.DERIV_H)
        return (t_high - t_low) / (2.0 * self.DERIV_H)

    def calculate(
        self, inputs: Optional[ShotInputs]
    ) -> "ShotCalculator.LaunchParameters":
        if (
            inputs is None
            or inputs.robotPose is None
            or inputs.fieldVelocity is None
            or inputs.robotVelocity is None
        ):
            return ShotCalculator.LaunchParameters.INVALID

        rawPose = inputs.robotPose
        fieldVel = inputs.fieldVelocity
        robotVel = inputs.robotVelocity

        poseX = rawPose.X()
        poseY = rawPose.Y()
        if math.isnan(poseX) or math.isnan(poseY) or math.isinf(poseX) or math.isinf(poseY):
            return ShotCalculator.LaunchParameters.INVALID

        dt = self.config.phaseDelayMs / 1000.0
        ax = (robotVel.vx - self.prevRobotVx) / 0.02
        ay = (robotVel.vy - self.prevRobotVy) / 0.02
        aOmega = (robotVel.omega - self.prevRobotOmega) / 0.02

        compensatedPose = rawPose.exp(
            Twist2d(
                robotVel.vx * dt + 0.5 * ax * dt * dt,
                robotVel.vy * dt + 0.5 * ay * dt * dt,
                robotVel.omega * dt + 0.5 * aOmega * dt * dt,
            )
        )

        self.prevRobotVx = robotVel.vx
        self.prevRobotVy = robotVel.vy
        self.prevRobotOmega = robotVel.omega

        robotX = compensatedPose.X()
        robotY = compensatedPose.Y()
        heading = compensatedPose.rotation().radians()

        hubCenter = inputs.hubCenter
        hubX = hubCenter.X()
        hubY = hubCenter.Y()

        hubForward = inputs.hubForward
        dot = (hubX - robotX) * hubForward.X() + (hubY - robotY) * hubForward.Y()
        if dot < 0.0:
            return ShotCalculator.LaunchParameters.INVALID

        if (
            abs(inputs.pitchDeg) > self.config.maxTiltDeg
            or abs(inputs.rollDeg) > self.config.maxTiltDeg
        ):
            return ShotCalculator.LaunchParameters.INVALID

        cosH = math.cos(heading)
        sinH = math.sin(heading)
        launcherX = robotX + self.config.launcherOffsetX * cosH - self.config.launcherOffsetY * sinH
        launcherY = robotY + self.config.launcherOffsetX * sinH + self.config.launcherOffsetY * cosH

        launcherFieldOffX = self.config.launcherOffsetX * cosH - self.config.launcherOffsetY * sinH
        launcherFieldOffY = self.config.launcherOffsetX * sinH + self.config.launcherOffsetY * cosH
        omega = fieldVel.omega
        vx = fieldVel.vx + (-launcherFieldOffY) * omega
        vy = fieldVel.vy + launcherFieldOffX * omega

        rx = hubX - launcherX
        ry = hubY - launcherY
        distance = math.hypot(rx, ry)

        if distance < self.config.minScoringDistance or distance > self.config.maxScoringDistance:
            return ShotCalculator.LaunchParameters.INVALID

        robotSpeed = math.hypot(vx, vy)
        if robotSpeed > self.config.maxSOTMSpeed:
            return ShotCalculator.LaunchParameters.INVALID

        velocityFiltered = robotSpeed < self.config.minSOTMSpeed

        solvedTOF: float
        projDist: float
        iterationsUsed: int
        warmStartUsed: bool

        if velocityFiltered:
            solvedTOF = self.effectiveTOF(distance)
            projDist = distance
            iterationsUsed = 0
            warmStartUsed = False
        else:
            maxIter = self.config.maxIterations
            convTol = self.config.convergenceTolerance

            if self.previousTOF > 0.0:
                tof = self.previousTOF
                warmStartUsed = True
            else:
                tof = self.effectiveTOF(distance)
                warmStartUsed = False

            projDist = distance
            iterationsUsed = 0

            for i in range(maxIter):
                prevTOF = tof

                c = self.config.sotmDragCoeff
                dragExp = 1.0 if c < 1e-6 else math.exp(-c * tof)
                driftTOF = tof if c < 1e-6 else (1.0 - dragExp) / c

                prx = rx - vx * driftTOF
                pry = ry - vy * driftTOF
                projDist = math.hypot(prx, pry)

                if projDist < 0.01:
                    tof = self.effectiveTOF(distance)
                    iterationsUsed = maxIter + 1
                    break

                lookupTOF = self.effectiveTOF(projDist)
                dPrime = -dragExp * (prx * vx + pry * vy) / projDist
                gPrime = self.tofMapDerivative(projDist)
                f = lookupTOF - tof
                fPrime = gPrime * dPrime - 1.0

                if abs(fPrime) > 0.01:
                    tof = tof - f / fPrime
                else:
                    tof = lookupTOF

                tof = _MathUtil.clamp(tof, self.config.tofMin, self.config.tofMax)
                iterationsUsed = i + 1

                if abs(tof - prevTOF) < convTol:
                    break

            if tof > self.config.tofMax or tof < 0.0 or math.isnan(tof):
                tof = self.effectiveTOF(distance)
                iterationsUsed = maxIter + 1

            solvedTOF = tof

        self.previousTOF = solvedTOF

        effectiveTOF = solvedTOF + self.config.mechLatencyMs / 1000.0
        effectiveRPMValue = self.effectiveRPM(projDist)

        if velocityFiltered:
            compTargetX = hubX
            compTargetY = hubY
        else:
            headingDriftTOF = self.dragCompensatedTOF(solvedTOF)
            compTargetX = hubX - vx * headingDriftTOF
            compTargetY = hubY - vy * headingDriftTOF

        aimX = compTargetX - robotX
        aimY = compTargetY - robotY
        driveAngle = Rotation2d(aimX, aimY)

        headingErrorRad = angleModulus(driveAngle.radians() - heading)

        driveAngularVelocity = 0.0
        if not velocityFiltered and distance > 0.1:
            tangentialVel = (ry * vx - rx * vy) / distance
            driveAngularVelocity = tangentialVel / distance

        if velocityFiltered:
            solverQuality = 1.0
        else:
            maxIter = self.config.maxIterations
            if iterationsUsed > maxIter:
                solverQuality = 0.0
            elif iterationsUsed <= 3:
                solverQuality = 1.0
            else:
                solverQuality = _MathUtil.interpolate(
                    1.0,
                    0.1,
                    float(iterationsUsed - 3) / float(maxIter - 3),
                )

        confidence = self.computeConfidence(
            solverQuality,
            robotSpeed,
            headingErrorRad,
            distance,
            inputs.visionConfidence,
        )

        self.previousSpeed = robotSpeed

        return ShotCalculator.LaunchParameters(
            effectiveRPMValue,
            effectiveTOF,
            driveAngle,
            driveAngularVelocity,
            True,
            confidence,
            distance,
            iterationsUsed,
            warmStartUsed,
        )

    def computeConfidence(
        self,
        solverQuality: float,
        currentSpeed: float,
        headingErrorRad: float,
        distance: float,
        visionConfidence: float,
    ) -> float:
        convergenceQuality = solverQuality

        speedDelta = abs(currentSpeed - self.previousSpeed)
        velocityStability = _MathUtil.clamp(1.0 - speedDelta / 0.5, 0.0, 1.0)

        visionConf = _MathUtil.clamp(visionConfidence, 0.0, 1.0)

        distanceScale = _MathUtil.clamp(
            self.config.headingReferenceDistance / distance,
            0.5,
            2.0,
        )
        speedScale = 1.0 / (1.0 + self.config.headingSpeedScalar * currentSpeed)
        scaledMaxError = self.config.headingMaxErrorRad * distanceScale * speedScale
        headingErr = abs(headingErrorRad)
        headingAccuracy = _MathUtil.clamp(1.0 - headingErr / scaledMaxError, 0.0, 1.0)

        rangeSpan = self.config.maxScoringDistance - self.config.minScoringDistance
        rangeFraction = (distance - self.config.minScoringDistance) / rangeSpan
        distInRange = 1.0 - 2.0 * abs(rangeFraction - 0.5)
        distInRange = _MathUtil.clamp(distInRange, 0.0, 1.0)

        c = [convergenceQuality, velocityStability, visionConf, headingAccuracy, distInRange]
        w = [
            self.config.wConvergence,
            self.config.wVelocityStability,
            self.config.wVisionConfidence,
            self.config.wHeadingAccuracy,
            self.config.wDistanceInRange,
        ]

        sumW = 0.0
        logSum = 0.0
        for i in range(5):
            if c[i] <= 0.0:
                return 0.0
            logSum += w[i] * math.log(c[i])
            sumW += w[i]

        if sumW <= 0.0:
            return 0.0

        composite = math.exp(logSum / sumW) * 100.0
        return _MathUtil.clamp(composite, 0.0, 100.0)

    def addRpmCorrection(self, distance: float, deltaRpm: float) -> None:
        self.correctionRpmMap.put(distance, deltaRpm)

    def addTofCorrection(self, distance: float, deltaTof: float) -> None:
        self.correctionTofMap.put(distance, deltaTof)

    def clearCorrections(self) -> None:
        self.correctionRpmMap.clear()
        self.correctionTofMap.clear()

    def adjustOffset(self, delta: float) -> None:
        self.rpmOffset = _MathUtil.clamp(self.rpmOffset + delta, -200.0, 200.0)

    def resetOffset(self) -> None:
        self.rpmOffset = 0.0

    def getOffset(self) -> float:
        return self.rpmOffset

    def getTimeOfFlight(self, distanceM: float) -> float:
        return self.effectiveTOF(distanceM)

    def getBaseRPM(self, distance: float) -> float:
        value = self.rpmMap.get(distance)
        if value is None:
            raise ValueError("RPM lookup table is empty")
        return value

    def resetWarmStart(self) -> None:
        self.previousTOF = -1.0
        self.previousSpeed = 0.0
        self.prevRobotVx = 0.0
        self.prevRobotVy = 0.0
        self.prevRobotOmega = 0.0

    def getRpmMap(self) -> _InterpolatingDoubleTreeMap:
        return self.rpmMap

    def getTofMap(self) -> _InterpolatingDoubleTreeMap:
        return self.tofMap


ShotCalculator.LaunchParameters.INVALID = ShotCalculator.LaunchParameters(
    0.0,
    0.0,
    Rotation2d(),
    0.0,
    False,
    0.0,
    0.0,
    0,
    False,
)
