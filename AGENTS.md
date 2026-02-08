# Repository Guidelines

## Project Structure & Module Organization
- Main robot code lives in `src/main/java/frc/robot/`.
- `subsystems/` contains hardware-facing subsystems (for example `SwerveSubsystem`, `IntakeSubsystem`).
- `commands/` contains command logic (Java and Kotlin examples exist).
- `RobotContainer.java` owns controller bindings and default commands.
- `Constants.java` is the central location for CAN IDs, limits, and tunables.
- Deploy-time assets (swerve JSON, other robot config files) live in `src/main/deploy/`.
- Vendor libraries are tracked in `vendordeps/`.

## Build, Test, and Development Commands
- `./gradlew build` compiles Java/Kotlin and packages the robot jar.
- `./gradlew test` runs JUnit 5 tests.
- `./gradlew deploy` builds and deploys to the configured roboRIO/team number.
- `./gradlew simulateJava` runs desktop simulation (when supported by current code/deps).
- `./gradlew tasks` lists available GradleRIO/WPILib tasks.
- Always make sure `./gradlew build` and `./gradlew test` both pass before finalizing changes.

## Coding Style & Naming Conventions
- Use Java 17 and Kotlin JVM 17 conventions.
- Indentation: 4 spaces; avoid tabs.
- Class names: `PascalCase`; methods/fields: `camelCase`; constants: `UPPER_SNAKE_CASE` or existing `kCamelCase` style used in `Constants`.
- Keep subsystem APIs small and command-friendly (`run`, `stop`, `toggle...`).
- Prefer placing operator mappings in `RobotContainer` rather than scattered in subsystem code.

## Testing Guidelines
- Framework: JUnit 5 (`useJUnitPlatform()` is enabled in `build.gradle`).
- Place tests under `src/test/java/...` (or `src/test/kotlin/...`) mirroring package paths.
- Test class naming: `<ClassName>Test` (example: `IntakeSubsystemTest`).
- Focus tests on command/subsystem behavior that can run off-robot; hardware integration should be verified on robot/sim.

## Commit & Pull Request Guidelines
- Use Conventional Commits format for general commit messages: `<type>(<scope>): <description>` (example: `feat(intake): add reverse control on left trigger`).
- Group related changes per commit; avoid mixing drivetrain, intake, and config edits unless tightly coupled.
- PRs should include what changed and why.
- PRs should include how it was verified (`./gradlew build`, sim run, on-robot check).
- PRs should call out any driver-control mapping changes clearly.
