# FRC Rebuilt 2025-2026

Java/Kotlin WPILib robot codebase using GradleRIO and YAGSL swerve.

## Project Layout
- `src/main/java/frc/robot/`: robot code (`Robot`, `RobotContainer`, `Constants`, subsystems, commands)
- `src/main/deploy/`: deploy-time assets, including swerve configuration JSON
- `vendordeps/`: vendor dependency definitions

## Build and Run
- `./gradlew build`: compile and package
- `./gradlew test`: run tests (JUnit 5)
- `./gradlew deploy`: deploy to roboRIO
- `./gradlew simulateJava`: run desktop simulation

## Controller Inputs and Behavior
Driver controller is an Xbox controller on port `0`.

- Left stick (`Y/X`): field-relative translation
- Right stick (`X`): rotation command (turn)
- Right trigger (> `0.1`): intake + indexer forward while held
- Left trigger (> `0.1`): intake + indexer reverse while held
- Trigger release: intake + indexer stop
- Intake/indexer speed changes are ramped (`kOutputRampRatePerSecond`) to reduce mechanical shock during direction changes
- Left bumper: toggle indexer inversion
- While indexer is inverted: subtle continuous right rumble (`kIndexerInvertedRumbleStrength`)
- Start button: zero gyro so current heading becomes straight ahead (alliance-aware)
- X button: wheel lock toggle
- Wheel lock ON: X-lock stance (hard to push)
- Wheel lock OFF: sets wheels back to an O-style turn pose, then normal driving continues

## Autonomous
Use SmartDashboard chooser:
- `Do Nothing` (default)
- `Drive Forward` (1 second)
