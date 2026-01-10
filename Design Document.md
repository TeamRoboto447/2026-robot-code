# FRC Robot Code Design: Architecture and Best Practices

## Overview

This document outlines the design principles and usage guidelines for our FRC robot code. It covers subsystems, commands, utility functions, constants, and overall project structure to ensure consistent, maintainable, and efficient code.

## Key Concepts and Terminology

Before diving into the specifics, here are some key concepts and terms used throughout this document:

- **API (Application Programming Interface)**: A set of defined methods that allow different parts of the code to communicate with each other.
- **Static Method**: A method that belongs to a class rather than an instance of the class. It can be called without creating an object of the class.
- **Instance**: A specific occurrence of a class, also known as an object.
- **Constructor**: A special method used to initialize objects. It's called when an object of a class is created.
- **Instantiation**: The process of creating an instance (object) of a class.
- **Extend**: In Java, this means a class is inheriting properties and methods from another class.
- **Override**: Redefining a method in a subclass that is already defined in its superclass.
- **Subsystem**: A major component of the robot, like the drivetrain or an arm mechanism.
- **Command**: A specific action or behavior that the robot can perform.
- **Periodic**: A method that runs repeatedly at a fixed time interval.

## Project Structure

Our project follows a standard WPILib command-based structure, with a clear organizational hierarchy:

- `src/main/java/frc/robot/`
  - `commands/`: Contains all command classes
  - `subsystems/`: Contains all subsystem classes
  - `utils/`: Contains utility classes and helper functions
  - `Constants.java`: Stores all constant values
  - `Robot.java`: Main robot class, This file is not to be touched
  - `RobotContainer.java`: Binds commands to inputs and initializes subsystems

## Subsystems

### Design Principles

1. **Hardware Abstraction**: Directly control motors, sensors, and physical components.
2. **State Management**: Track current and desired subsystem states.
3. **Safety Features**: Implement safeguards like limit switches and current monitoring.
4. **API Design**: Provide clear, high-level methods for commands to use.

### Implementation Guidelines

- Extend the `SubsystemBase` class
- Initialize hardware components in the constructor
- Implement `periodic()` method for continuous updates and safety checks
- Provide public methods that offer a clean API for commands
- Keep hardware-specific details private within the subsystem

## Commands

### Design Principles

1. **Single Responsibility**: Perform one specific action
2. **Subsystem Interaction**: Use subsystem APIs, never directly control hardware
3. **Input Handling**: Process inputs from operators, sensors, or other sources
4. **State Transitions**: Clearly define command start, end, and interruption conditions

### Implementation Guidelines

- Extend the `CommandBase` class
- Declare required subsystems in the constructor using `addRequirements()`
- Implement `initialize()`, `execute()`, `end()`, and `isFinished()` methods as needed
- Use `execute()` for continuous actions and checks
- Return `true` in `isFinished()` when the command's goal is achieved

## Constants

### Guidelines

1. Store all configuration values in `Constants.java`
2. Organize constants into nested classes based on subsystem or functionality
3. Use UPPER_SNAKE_CASE for all constant values
4. Provide clear, descriptive names

## Utility Functions

### Guidelines

1. Create separate utility classes for distinct functionality
2. Make utility methods static for easy access
3. Provide clear documentation for each method
4. Keep utility functions focused and single-purpose
5. Prevent instantiation of utility classes

## RobotContainer

### Responsibilities

1. Initialize all subsystems
2. Configure default commands for subsystems
3. Set up button bindings for operator interface
4. Create methods for autonomous command selection
5. Initialize and manage different controller types
6. Handle file system operations for configuration
7. Initialize multi-system commands

### Implementation Guidelines

1. Create separate initialization methods for each subsystem alongside commands that only interact with it
2. Set default commands for subsystems that require continuous operation
3. Use appropriate controller classes (e.g., CommandJoystick, CommandXboxController)
4. Implement `getAutonomousCommand()` to return the requested autonomous routine
5. Commands that interact with multiple subsystems should be initialized in `initializeMultisystemCommands`

## Best Practices

1. **Separation of Concerns**: Maintain clear boundaries between subsystems, commands, and utilities
2. **Reusability**: Design components to be as flexible as possible
3. **Testability**: Structure code to support unit testing
4. **Documentation**: Thoroughly comment complex logic
5. **Consistent Naming**: Use clear, consistent naming conventions
6. **Version Control**: Make frequent, small, well-described commits
7. **Code Review**: Implement peer review for all code changes

## Version Control and Git Workflow

Our team uses a structured Git workflow to manage code contributions and maintain code quality.

### Branch Structure

1. **main**: Contains the latest stable, tested code. Only mentors may modify this branch.
2. **development**: Contains the latest integrated features and updates. Completed features are merged here.
3. **feature branches**: Individual branches for work-in-progress features, created from `development`.

### Contributing Code

1. **Create a feature branch** from `development`:
   ```bash
   git checkout development
   git pull origin development
   git checkout -b feature/your-feature-name
   ```

2. **Develop your feature**: Make regular commits with clear, descriptive messages that explain what changed and why.

3. **Push your branch** to GitHub:
   ```bash
   git push origin feature/your-feature-name
   ```

4. **Submit a pull request** to merge into `development`. Request a review from a mentor or team lead.

5. **Code review**: Address feedback and make additional commits as needed.

6. **Merge**: Once approved, a mentor will merge your pull request into `development`.

### Commit Message Guidelines

- Use present tense ("Add feature" not "Added feature")
- Be descriptive but concise
- Reference issue numbers when applicable
- Examples:
  - "Add intake subsystem with roller control"
  - "Fix drivetrain motor inversion issue"
  - "Update shooter constants for competition field"

## Development Workflow

1. Identify robot functionalities and create GitHub issues or tasks
2. Create a feature branch for the task from `development`
3. Design and create corresponding subsystems
4. Implement subsystem methods and APIs
5. Develop commands using subsystem APIs
6. Create utility functions for common operations
7. Configure `RobotContainer` with appropriate bindings
8. Test thoroughly on the robot
9. Commit changes with clear messages
10. Submit pull request for code review
11. Address review feedback and iterate
12. Merge approved code into `development`

By following these guidelines, we ensure a modular, maintainable, and efficient robot code structure.
