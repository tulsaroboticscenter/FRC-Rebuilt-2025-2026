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

## PathPlanner Setup
Build and run the PathPlanner desktop app locally:

macOS prerequisites (one-time setup):
- `sudo xcode-select --switch /Applications/Xcode.app/Contents/Developer`
- `sudo xcodebuild -runFirstLaunch`
- `brew install cocoapods`

1. Clone the repo:
   - `git clone https://github.com/mjansen4857/pathplanner.git`
   - `cd pathplanner`
2. Install Flutter from https://docs.flutter.dev/get-started/install and verify with `flutter doctor`.
   - Homebrew option: `brew install --cask flutter`
3. Build the app:
   - macOS: `flutter build macos`
   - Windows: `flutter build windows`
   - Linux: `flutter build linux`
4. Run the app:
   - Debug mode: `flutter run`
   - macOS release build: `open build/macos/Build/Products/Release/PathPlanner.app`

## Controller Inputs and Behavior
Driver controller is a PS5 controller on port `0`.
Manipulator controller is an Xbox controller on port `1`.

Driver controls:
- Left stick (`Y/X`): field-relative translation
- Right stick (`X`): rotation command
- `Triangle`: swerve brake
- `L1`: reseed field-centric heading
- `R1`: Limelight angle-only assist while preserving translational control
- `R2`: feeder in
- `L2`: feeder out

Manipulator controls:
- `Right trigger`: shooter forward at the current target speed
- `Left bumper`: shooter reverse
- `Left trigger`: intake forward
- `Right bumper`: intake reverse
- `D-pad right`: increase shooter target by `1 RPS`
- `D-pad left`: decrease shooter target by `1 RPS`
- `D-pad down`: toggle shooter target between reduced mode and full-speed mode

Shooter tuning and feedback:
- Reduced shooter target defaults to `52.0 RPS` (50%)
- Full shooter target is `104.0 RPS` (100%)
- `Shooter/TargetRPS` on SmartDashboard can still override the live target
- When full-speed mode is active, the manipulator controller pulses rumble at 50% strength once per second
