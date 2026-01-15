package frc.robot.commands;

public class JoystickDrive extends Command {
    private SwerveDrive drive;
    ... // Other requirements

    public JoystickDrive(
            SwerveDrive drive, // Take an instance of the SwerveDrive interface as a dependency
            ... // Other requirements (e.g., driver gamepad axis suppliers)
    ) {
        this.drive = drive;
        addRequirements(drive);
        ... // Process other requirements
    }

    @Override
    public void execute() {
        ... // Process command logic

        drive.drive(...); // Execute the logic through APIs specified by the SwerveDrive interface
    }
}