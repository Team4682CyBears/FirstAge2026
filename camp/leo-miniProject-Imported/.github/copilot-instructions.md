<!-- .github/copilot-instructions.md - guidance for AI coding agents -->
# Copilot / AI agent instructions — FirstAge2026 / leo-miniProject-Imported

Keep this short and specific. Prefer making small, local changes and referencing existing files.

- Project type: Java (WPILib FRC robot) using Gradle wrapper. Key source: `src/main/java/frc/robot`.
- Build: use the included Gradle wrapper from project root. On Windows PowerShell:
  - `.\\gradlew.bat build` (or `.\\gradlew build`)
  - WPILib tasks (deploy, simulate) are available via the wrapper (e.g. `deploy`).

What to know up front
- Command framework: this repo uses WPILib's Command-based pattern. Look at `Robot.java` and
  `RobotContainer.java` to understand lifecycle and trigger-to-command mappings.
- Constants-only policy: `Constants.java` is for global numeric/boolean constants only — do not
  add logic there.
- Code layout conventions:
  - Subsystems live under `src/main/java/frc/robot/subsystems`.
  - Reusable hardware wrappers are in `src/main/java/frc/robot/common` (example: `TofSensorPWF.java`).
  - `src/main/deploy` contains deployment assets.

Common patterns / examples
- Scheduler and lifecycle: `Robot.robotPeriodic()` calls `CommandScheduler.getInstance().run()` —
  ensure command scheduling behavior by adding or removing commands via `RobotContainer`.
- Sensor wrappers: `TofSensorPWF` wraps PlayingWithFusion TimeOfFlight usage (units via `Units`,
  telemetry via `SmartDashboard`). Follow the same pattern for other sensors: constructor sets
  hardware, provide small public getters, and a `publishTelemetry()` method.
- Use `Constants` for values like `MAX_RANGE_METERS`, CAN IDs, and controller ports.

Integration points and external deps
- Vendor dependencies live in `vendordeps/` (e.g. `Phoenix6-26.1.0.json`, `playingwithfusion.json`)
  — these are WPILib vendor manifests. Do not edit them unless updating vendor libs.
- CTRE / Phoenix and PlayingWithFusion libraries are used; match their types in wrappers under
  `common/` (e.g. `TofSesorCTRE.java`, `TofSensorPWF.java`).

Developer workflows
- Build locally with the Gradle wrapper. On Windows PowerShell from repo root:
  - `.\\gradlew.bat build` — compiles and packages the robot code.
  - `.\\gradlew.bat deploy` — (WPILib) deploy to robot/simulator if configured.
- Debugging: prefer small, targeted runs. Add SmartDashboard telemetry via `SmartDashboard.putNumber/Boolean/String`.

Style & acceptable changes for AI agents
- Make minimal, testable changes. Follow the Command-based architecture: prefer adding commands and
  subsystems over putting logic in `Robot` or `Constants`.
- When introducing new public APIs, update one example usage (e.g., in `RobotContainer`) so reviewers
  can see intended usage.
- Avoid modifying vendor manifests in `vendordeps/` unless explicitly requested.

If you need more context
- Inspect these files first:
  - `src/main/java/frc/robot/Robot.java`
  - `src/main/java/frc/robot/RobotContainer.java`
  - `src/main/java/frc/robot/Constants.java`
  - `src/main/java/frc/robot/common/TofSensorPWF.java`
- If a change affects hardware or deployment, ask before modifying `deploy/` or vendor JSONs.

Done: create small, explicit PRs with 1–3 changes and a clear description referencing the files changed.

Questions or missing details? Ask the repo owner for hardware mapping, CI rules, or preferred deploy targets.
