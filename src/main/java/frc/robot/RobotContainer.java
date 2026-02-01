package frc.robot;

import edu.wpi.first.math.estimator.PoseEstimator;

// import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.io.File;
import java.util.Optional;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import swervelib.SwerveInputStream;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.miscConstants;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Swerve.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private static Optional<DriverStation.Alliance> lastAlliance = Optional.empty();
    final         CommandXboxController DriveController = new CommandXboxController(0);
    // The robot's subsystems and commands are defined here...
    private final static SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                      "swerve"));
                          
      private SendableChooser<Command> autoChooser;
      
      
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                   () -> DriveController.getLeftY() * -1,
                                                                   () -> DriveController.getLeftX() * -1)
                                                                  //.withControllerRotationAxis(() -> DriveController.getRawAxis(2))
                                                                  .withControllerRotationAxis(DriveController::getRightX)
                                                                  .deadband(miscConstants.DEADBAND)
                                                                  .scaleTranslation(0.20)
                                                                  .scaleRotation(0.15)
                                                                  .allianceRelativeControl(true);
                  
       
      
        /**
         * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
         */
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(DriveController::getLeftX,
                                                                                                   DriveController::getLeftY)
                                                                 .headingWhile(true);
                                                                 
      
        /**
         * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
         */
        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(false)
                                                                   .allianceRelativeControl(true);
      
        SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                            () -> -DriveController.getLeftY(),
                                                                            () -> -DriveController.getLeftX())
                                                                          .withControllerRotationAxis(() -> DriveController.getRawAxis(
                                                                              // () -> -driverJoystick.getRawAxis(1),
                                                                              // () -> -driverJoystick.getRawAxis(0))
                                                                          // .withControllerRotationAxis(() -> driverJoystick.getRawAxis(
                                                                              2))
                                                                          .deadband(miscConstants.DEADBAND)
                                                                          .scaleTranslation(0.8)
                                                                          .allianceRelativeControl(true);
        // Derive the heading axis with math!
        SwerveInputStream driveDirectAngleKeyboard     = driveAngularVelocityKeyboard.copy()
                                                                                     .withControllerHeadingAxis(() ->
                                                                                                                    Math.sin(
                                                                                                                      DriveController.getRawAxis(
                                                                                                                        // driverJoystick.getRawAxis(
                                                                                                                            2) *
                                                                                                                        Math.PI) *
                                                                                                                    (Math.PI *
                                                                                                                     2),
                                                                                                                () ->
                                                                                                                    Math.cos(
                                                                                                                      DriveController.getRawAxis(
                                                                                                                        // driverJoystick.getRawAxis(
                                                                                                                            2) *
                                                                                                                        Math.PI) *
                                                                                                                    (Math.PI *
                                                                                                                     2))
                                                                                     .headingWhile(true);
      
        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
          
          configureBindings();
          DriverStation.silenceJoystickConnectionWarning(true);
      
      }
      
      


//======================================================================================
  
  private void configureBindings()
  {
        

 Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

 Command driveFieldOrientedDirectAngleKeyboard  = drivebase.driveFieldOriented(driveDirectAngleKeyboard);


   if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      DriveController.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      DriveController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }

      //~~~~~~~~~~~~~~~~~~Drive Control~~~~~~~~~~~~~~~~~~~~~~~~
      DriveController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      
      DriveController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));


      
      //Go to pos ?
       //NOTE - Pos not right
      // DriveController.b().whileTrue(
      //   drivebase.driveToPose(new Pose2d(new Translation2d(14.4, 4.0), Rotation2d.fromDegrees(180))));


      //This is our boost control Right Trigger
      DriveController.axisGreaterThan(3, 0.01).onChange(Commands.runOnce(() -> {
        driveAngularVelocity.scaleTranslation(DriveController.getRightTriggerAxis() + 0.35);
        driveAngularVelocity.scaleRotation((DriveController.getRightTriggerAxis() * miscConstants.RotationSpeedScale) + 0.25);
      }).repeatedly()).whileFalse(Commands.runOnce(() -> { 
        driveAngularVelocity.scaleTranslation(0.25);
        driveAngularVelocity.scaleRotation(0.15);
      }).repeatedly());

       }



    
  

  public void setupAutonomous() {
        // Named Commands go here
        //NamedCommands.registerCommand("GUI NAME", theCommand());
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(autoChooser);
    }

    /**
     * Gets the selected autonomous command.
     *
     * @return the selected {@link Command}.
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}