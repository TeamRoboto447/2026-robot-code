# Team Roboto 447 - FRC 2026 Rebuilt

Welcome to Team Roboto 447's repository for the FIRST Robotics Competition 2026 season!

## About the Game

### Match Overview

Each Match lasts 2 minutes and 40 seconds, with two Alliances of three teams competing to earn points by scoring Fuel in their active Hub and climbing their Tower. Robots start on their Starting Line and may be preloaded with up to eight Fuel.

### Autonomous Period (20 seconds)

- Both Hubs are Active during Auto
- Robots operate autonomously to:
  - Score Fuel into the Hub (1 point each)
  - Climb their Tower to Level 1
- Additional Fuel can be collected from the Depot or from Human Players in the Outpost
- Human Players can throw from the Outpost into the Hub to score points
- Scored Fuel is returned to the Neutral Zone

### Teleoperated Period (2 minutes 20 seconds)

Teleop is divided into six segments where Drivers control their Robots:

1. **Transition Shift (10 seconds)**: Both Hubs are Active
2. **Alliance Shift 1 (25 seconds)**: The Alliance that scored the most Fuel in Auto will have their Hub Inactive; the opposing Hub is Active
3. **Alliance Shift 2 (25 seconds)**: Hub status switches
4. **Alliance Shift 3 (25 seconds)**: Hubs alternate again
5. **Alliance Shift 4 (25 seconds)**: Hubs alternate again
6. **End Game (30 seconds)**: Both Hubs are Active

During Teleop, each Fuel scored in an Active Hub is worth 1 point.

### Field Elements

- **Hubs**: Active or Inactive depending on the shift; Alliances score Fuel here
- **Tower**: Climbing structure with three levels
- **Depot**: Source of additional Fuel
- **Outpost**: Where Human Players can deliver Fuel to Robots or throw into the Hub
- **Neutral Zone**: Area where Fuel is returned after scoring
- **Bumps and Trenches**: Obstacles Robots can drive over or under to access the Neutral Zone

### Scoring & Ranking Points

**Fuel Scoring:**
- Auto: 1 point per Fuel
- Teleop: 1 point per Fuel in Active Hub

**Climbing:**
- Level 1: 10 points
- Level 2: 20 points
- Level 3: 30 points

**Ranking Points:**
- **Energized**: Score enough Fuel to meet the Energized threshold (100 for regional/district events)
- **Supercharged**: Score enough Fuel to meet the Supercharged threshold (360 for regional/district events)
- **Traversal**: Earn enough Tower points from climbing in Auto or Teleop (50 for regional/district events)

The Alliance with the most points overall wins the Match.

## Code Structure

Our robot code is built using Java and the WPILib framework. Here's a basic overview of our code structure:

- `src/main/java/frc/robot/`: Main source code directory
  - `commands/`: Contains command classes for robot actions
  - `subsystems/`: Contains subsystem classes for major robot components
  - `utils/`: Contains utility classes and helper functions
  - `Constants.java`: Stores important constant values
  - `Robot.java`: Main robot class
  - `RobotContainer.java`: Configures and binds commands to inputs

## Getting Started with WPILib

To work on this project, you'll need to install the WPILib development environment:

1. Download the latest WPILib installer from the [WPILib Releases page](https://github.com/wpilibsuite/allwpilib/releases).
2. Choose the appropriate installer for your operating system (Windows, macOS, or Linux).
3. Run the installer and follow the prompts to set up WPILib and Visual Studio Code.

For detailed installation instructions, refer to the [WPILib Installation Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html).

## Contributing Code

As a student on Team Roboto 447, here's how you should contribute code:

1. **Main Branch**: Contains the latest stable, tested code. Only mentors may modify this branch in any way.
2. **Development Branch**: Contains the latest updates and features that are being integrated. This is where completed features are merged.
3. **Feature Branches**: Create a new branch for each feature or task you're working on. Branch off from `development` and name your branch descriptively (e.g., `feature/intake-mechanism` or `fix/drivetrain-bug`).

### Workflow for Students

1. **Start a new feature**: Create a new feature branch from the `development` branch.
   ```bash
   git checkout development
   git pull origin development
   git checkout -b feature/your-feature-name
   ```

2. **Work on your feature**: Make your code changes and commit them regularly with clear, descriptive commit messages.

3. **Push your changes**: Push your feature branch to GitHub.
   ```bash
   git push origin feature/your-feature-name
   ```

4. **Submit a pull request**: When your feature is complete and tested, create a pull request to merge your feature branch into `development`. Request a review from a mentor or team lead.

5. **Code review**: Address any feedback from the code review process. Make additional commits to your feature branch as needed.

6. **Merge**: Once approved, a mentor will merge your pull request into the `development` branch.

Remember to always communicate with the team about the changes you're making and ask for help if you're unsure about anything!
