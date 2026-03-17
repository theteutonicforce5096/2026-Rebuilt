# frc-fire-control

Shoot-on-the-move fire control for FRC. Three standalone Java files, MIT licensed, no team-specific dependencies. Just needs `wpimath` and `ntcore` which you already have from GradleRIO.

[![Demo](https://img.youtube.com/vi/hWCm1sssc6Q/maxresdefault.jpg)](https://www.youtube.com/watch?v=hWCm1sssc6Q)

## What's in here

- **ShotCalculator** - Newton-method SOTM solver. Tells your robot what RPM and heading it needs while driving. Handles launcher offset, latency compensation, drag on the ball, and gives you a 0-100 confidence score so you know when it's actually safe to fire.
- **ProjectileSimulator** - RK4 projectile sim with drag and Magnus lift. Binary-searches RPM for each distance and gives you a 91-point lookup table. You plug in your robot's measurements from CAD and it does the rest. No more hand-tuning shooter tables from match videos.
- **FuelPhysicsSim** - Full-field ball physics for simulation. Drag, Magnus, friction, ball-ball collisions, wall bounces, hub scoring, CCD, sleeping. Drop it into your sim and watch balls fly around the field. ~2300 lines but it's one file with zero external dependencies beyond WPILib. **Note:** the field geometry (hub positions, bumps, trenches, nets, towers) is hardcoded for 2026 REBUILT. If you want to use this for a different game, you'd need to redo the field constants.

## How to use it

Grab the 3 Java files from `src/main/java/frc/firecontrol/` and drop them into your project. Change the package declaration to match yours. That's it.

They only depend on `wpimath` and `ntcore`, which GradleRIO already gives you.

## Version tracking

We tag releases on GitHub (v1.0.0, v1.1.0, etc.) with release notes explaining what changed and what breaks. If you want to know when we push updates, click "Watch" > "Releases only" on this repo and GitHub will email you.

## Quick start

### Generate a shooter lookup table

```java
// plug in YOUR robot's measurements from CAD
ProjectileSimulator.SimParameters params = new ProjectileSimulator.SimParameters(
    0.215,   // ball mass kg
    0.1501,  // ball diameter m
    0.47,    // drag coeff (smooth sphere)
    0.2,     // Magnus coeff
    1.225,   // air density
    0.43,    // exit height (m), floor to where the ball leaves the shooter
    0.1016,  // flywheel diameter (m), measure with calipers
    1.83,    // target height (m), from game manual
    0.6,     // slip factor (0=no grip, 1=perfect), tune this on the real robot
    45.0,    // launch angle from horizontal, measure from CAD
    0.001,   // sim timestep
    1500, 6000, 25, 5.0  // RPM search range, iterations, max sim time
);

ProjectileSimulator sim = new ProjectileSimulator(params);
ProjectileSimulator.GeneratedLUT lut = sim.generateLUT();

// print it out
for (var entry : lut.entries()) {
    if (entry.reachable()) {
        System.out.printf("%.2fm -> %.0f RPM, %.3fs TOF%n",
            entry.distanceM(), entry.rpm(), entry.tof());
    }
}
```

### Wire up shoot-on-the-move

```java
// in RobotContainer or wherever you set stuff up
ShotCalculator.Config config = new ShotCalculator.Config();
config.launcherOffsetX = 0.23;  // how far forward the launcher is from robot center (m)
config.launcherOffsetY = 0.0;   // how far left, 0 if centered
config.phaseDelayMs = 30.0;     // your vision pipeline latency
config.mechLatencyMs = 20.0;    // how long the mechanism takes to respond
config.maxTiltDeg = 5.0;        // suppress firing when chassis tilts past this (bumps/ramps)
config.headingSpeedScalar = 1.0; // heading tolerance tightens with robot speed (0 to disable)
config.headingReferenceDistance = 2.5; // heading tolerance scales with distance from hub

ShotCalculator shotCalc = new ShotCalculator(config);

// load the LUT you generated
for (var entry : lut.entries()) {
    if (entry.reachable()) {
        shotCalc.loadLUTEntry(entry.distanceM(), entry.rpm(), entry.tof());
    }
}

// call this every cycle in robotPeriodic()
Translation2d hubCenter = new Translation2d(4.6, 4.0);  // your target
Translation2d hubForward = new Translation2d(1, 0);       // which way the hub faces

ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
    swerve.getPose(),
    swerve.getFieldVelocity(),
    swerve.getRobotVelocity(),
    hubCenter,
    hubForward,
    0.9,  // vision confidence, 0 to 1
    swerve.getPitch().getDegrees(),  // pitch for tilt gate (0.0 if no gyro)
    swerve.getRoll().getDegrees()    // roll for tilt gate (0.0 if no gyro)
);

ShotCalculator.LaunchParameters shot = shotCalc.calculate(inputs);
if (shot.isValid() && shot.confidence() > 50) {
    shooter.setRPM(shot.rpm());
    drivebase.aimAt(shot.driveAngle());
    // shot.driveAngularVelocityRadPerSec() gives you a heading feedforward if you want it
}
```

### Let your copilot trim RPM during a match

```java
// bind to copilot D-pad
copilotController.povUp().onTrue(Commands.runOnce(() -> shotCalc.adjustOffset(25)));
copilotController.povDown().onTrue(Commands.runOnce(() -> shotCalc.adjustOffset(-25)));
// reset on mode change so trim doesn't carry over
shotCalc.resetOffset();
```

### Ball physics sim

```java
// in simulationInit()
FuelPhysicsSim ballSim = new FuelPhysicsSim("Sim/Fuel");
ballSim.enable();
ballSim.placeFieldBalls();  // spawns all the game pieces

// tell it about your robot
ballSim.configureRobot(robotWidth, robotLength, bumperHeight,
    () -> swerve.getPose(), () -> swerve.getChassisSpeeds());

// in simulationPeriodic()
ballSim.tick();  // runs physics, publishes ball positions to NT

// when you shoot
ballSim.launchBall(launcherPosition, launchVelocity, spinRPM);
```

Ball positions publish to NetworkTables so you can see them in AdvantageScope's Field3d view.

## How the math works

### ShotCalculator

If you're driving at velocity **v** and the ball takes time **t** to reach the hub, you need to aim at where the hub will "appear to be" after accounting for your motion during flight. But **t** depends on the aim distance, which depends on **t**. It's circular, so we treat it as root-finding: define f(t) = TOF_from_LUT(projected_distance(t)) - t, and Newton's method finds f(t) = 0 in 2-3 iterations. We warm-start from last cycle's answer so it usually converges in 1.

Drag compensation adjusts for the ball's inherited velocity decaying during flight. The real displacement is (1 - e^(-c*t)) / c instead of v*t. Matters more at longer ranges.

The confidence score is a weighted geometric mean of 5 things: solver convergence, velocity stability, vision confidence, heading accuracy, and distance from range edges. If any one drops to zero, the whole score goes to zero. That's intentional, you shouldn't shoot if any single factor is completely gone. Heading tolerance tightens at higher speeds (velocity errors compound harder) and at closer range (a 5-degree error at 1.5m misses by way more than at 4m). The solver also suppresses firing when the chassis is tilted past 5 degrees because bumps knock the launcher off-axis.

Pose prediction uses second-order (`v*dt + 0.5*a*dt^2`) instead of first-order. The acceleration comes from the velocity delta between this cycle and last, so it tracks through turns better.

### ProjectileSimulator

RK4 integration with quadratic drag and Magnus lift. For each distance, binary search finds the RPM where the ball arrives at the target height within 2cm. 25 iterations gives ~0.004 RPM precision, way more than you need.

### FuelPhysicsSim

Symplectic Euler with 3D angular velocity (omega x v for Magnus). Sequential impulse collision solver with warm starting. Spatial hashing for ball-ball broadphase. Ball sleeping so 350+ resting balls cost under 2ms/tick. Full 2026 REBUILT field geometry with per-material COR values.

## Constants you need to measure

| What | How to get it |
|------|---------------|
| Exit height | CAD: floor to ball center at the moment it leaves the shooter |
| Launch angle | CAD: angle of the exit trajectory from horizontal |
| Wheel diameter | Calipers on the flywheel |
| Slip factor | Measure actual ball speed vs wheel surface speed, usually 0.5-0.7 |
| Launcher offset X/Y | CAD: distance from robot center to where the ball exits |

## Discussion

Questions, feedback, or want to show off your SOTM shots? Post in the [Chief Delphi thread](https://www.chiefdelphi.com/t/open-source-shoot-on-the-move-sotm-solver-ball-physics-sim-3-java-files-drop-in/).

## License

MIT. Use it, change it, share it. If it helps your team score more, that's all we wanted.

FRC Team 5962 perSEVERE, 2026 season.
