# Team Roboto 447 - FRC 2026 Rebuilt

Welcome to Team Roboto 447's repository for the FIRST Robotics Competition 2026 season!

## About the Game

The FRC 2026 game will be revealed during the Kickoff event. At this time, we know that each match will consist of:

- **Autonomous Period**: 15 seconds
- **Teleoperated Period**: 2 minutes and 15 seconds

Additional game details, including objectives, scoring mechanisms, and field elements, will be announced at Kickoff. This README will be updated once the game is revealed.

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
