package frc.robot.subsystems;

import choreo.auto.AutoChooser;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.team190.gompeilib.core.GompeiLib.Mode;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIO;
import edu.wpi.team190.gompeilib.core.io.components.inertial.GyroIOPigeon2;
import edu.wpi.team190.gompeilib.core.robot.RobotContainer;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDrive;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIO;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOSim;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveModuleIOTalonFX;
import edu.wpi.team190.gompeilib.subsystems.vision.Vision;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraLimelight;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
import frc.robot.Constants;
import frc.robot.commands.DriveCommands;
import java.util.List;

public class ExampleRobotRobotContainer implements RobotContainer {
  private SwerveDrive drive;
  private Vision vision;

  private final CommandXboxController driver = new CommandXboxController(0);

  private final AutoChooser autoChooser = new AutoChooser();

  public ExampleRobotRobotContainer() {
    if (Constants.getMode() != Mode.REPLAY) {
      switch (Constants.ROBOT) {
        case EXAMPLE_ROBOT:
          drive =
              new SwerveDrive(
                  ExampleRobotConstants.DRIVE_CONSTANTS,
                  new GyroIOPigeon2(ExampleRobotConstants.DRIVE_CONSTANTS),
                  new SwerveModuleIOTalonFX(
                      ExampleRobotConstants.DRIVE_CONSTANTS,
                      ExampleRobotConstants.DRIVE_CONSTANTS.FRONT_LEFT),
                  new SwerveModuleIOTalonFX(
                      ExampleRobotConstants.DRIVE_CONSTANTS,
                      ExampleRobotConstants.DRIVE_CONSTANTS.FRONT_RIGHT),
                  new SwerveModuleIOTalonFX(
                      ExampleRobotConstants.DRIVE_CONSTANTS,
                      ExampleRobotConstants.DRIVE_CONSTANTS.BACK_LEFT),
                  new SwerveModuleIOTalonFX(
                      ExampleRobotConstants.DRIVE_CONSTANTS,
                      ExampleRobotConstants.DRIVE_CONSTANTS.BACK_RIGHT),
                  () -> Pose2d.kZero,
                  ExampleRobotRobotState::resetPose);
          vision =
              new Vision(
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark),
                  new CameraLimelight(
                      new CameraIOLimelight(ExampleRobotConstants.LIMELIGHT_CONFIG),
                      ExampleRobotConstants.LIMELIGHT_CONFIG,
                      ExampleRobotRobotState::getHeading,
                      NetworkTablesJNI::now,
                      List.of(),
                      List.of()));
          break;

        case EXAMPLE_ROBOT_SIM:
          drive =
              new SwerveDrive(
                  ExampleRobotConstants.DRIVE_CONSTANTS,
                  new GyroIO() {},
                  new SwerveModuleIOSim(
                      ExampleRobotConstants.DRIVE_CONSTANTS,
                      ExampleRobotConstants.DRIVE_CONSTANTS.FRONT_LEFT),
                  new SwerveModuleIOSim(
                      ExampleRobotConstants.DRIVE_CONSTANTS,
                      ExampleRobotConstants.DRIVE_CONSTANTS.FRONT_RIGHT),
                  new SwerveModuleIOSim(
                      ExampleRobotConstants.DRIVE_CONSTANTS,
                      ExampleRobotConstants.DRIVE_CONSTANTS.BACK_LEFT),
                  new SwerveModuleIOSim(
                      ExampleRobotConstants.DRIVE_CONSTANTS,
                      ExampleRobotConstants.DRIVE_CONSTANTS.BACK_RIGHT),
                  () -> Pose2d.kZero,
                  ExampleRobotRobotState::resetPose);
          vision =
              new Vision(
                  () -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark));
          break;

        default:
          break;
      }
    }
    if (drive == null) {
      drive =
          new SwerveDrive(
              ExampleRobotConstants.DRIVE_CONSTANTS,
              new GyroIOPigeon2(ExampleRobotConstants.DRIVE_CONSTANTS),
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              new SwerveModuleIO() {},
              () -> Pose2d.kZero,
              ExampleRobotRobotState::resetPose);
    }

    if (vision == null) {
      new Vision(() -> AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark));
    }

    configureButtonBindings();
    configureAutos();
  }

  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            ExampleRobotConstants.DRIVE_CONSTANTS,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            ExampleRobotRobotState::getHeading));
  }

  private void configureAutos() {
    // Autos here
  }

  @Override
  public void robotPeriodic() {
    ExampleRobotRobotState.periodic();
  }

  @Override
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }
}
