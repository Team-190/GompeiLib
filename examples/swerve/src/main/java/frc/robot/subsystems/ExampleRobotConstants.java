package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.AutoAlignNearConstants;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.AutoGains;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.DriveConfig;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.Gains;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants.PIDControllerConstants;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants.LimelightConfig;
import edu.wpi.team190.gompeilib.subsystems.vision.camera.CameraType;

public class ExampleRobotConstants {
  public static final DriveConfig DRIVE_CONFIG =
      new DriveConfig(
          new CANBus("Drive"),
          1,
          5.0,
          Units.inchesToMeters(2.0),
          DCMotor.getKrakenX60Foc(1),
          DCMotor.getKrakenX44Foc(1),
          TunerConstantsExampleRobot.FrontLeft,
          TunerConstantsExampleRobot.FrontRight,
          TunerConstantsExampleRobot.BackLeft,
          TunerConstantsExampleRobot.BackRight,
          25.0,
          25.0);
  public static final Gains GAINS =
      new Gains(
          new LoggedTunableNumber("Drive/Teleop/Drive Ks", 0.0),
          new LoggedTunableNumber("Drive/Teleop/Drive Kv", 0.0),
          new LoggedTunableNumber("Drive/Teleop/Drive Kp", 0.0),
          new LoggedTunableNumber("Drive/Teleop/Drive Kd", 0.0),
          new LoggedTunableNumber("Drive/Teleop/Turn Kp", 0.0),
          new LoggedTunableNumber("Drive/Teleop/Turn Kd", 0.0));
  public static final AutoGains AUTO_GAINS =
      new AutoGains(
          new LoggedTunableNumber("Drive/Auto/Translation Kp", 0.0),
          new LoggedTunableNumber("Drive/Auto/Translation Kd", 0.0),
          new LoggedTunableNumber("Drive/Auto/Rotation Kp", 0.0),
          new LoggedTunableNumber("Drive/Auto/Rotation Kd", 0.0));
  public static final AutoAlignNearConstants AUTO_ALIGN_NEAR_CONSTANTS =
      new AutoAlignNearConstants(
          new PIDControllerConstants(
              new LoggedTunableNumber("Drive/Auto Align/X/Kp", 0.0),
              new LoggedTunableNumber("Drive/Auto Align/X/Kd", 0.0),
              new LoggedTunableNumber("Drive/Auto Align/X/Tolerance", 0.0),
              new LoggedTunableNumber("Drive/Auto Align/X/Max Velocity", 0.0)),
          new PIDControllerConstants(
              new LoggedTunableNumber("Drive/Auto Align/Y/Kp", 0.0),
              new LoggedTunableNumber("Drive/Auto Align/Y/Kd", 0.0),
              new LoggedTunableNumber("Drive/Auto Align/Y/Tolerance", 0.0),
              new LoggedTunableNumber("Drive/Auto Align/Y/Max Velocity", 0.0)),
          new PIDControllerConstants(
              new LoggedTunableNumber("Drive/Auto Align/Theta/Kp", 0.0),
              new LoggedTunableNumber("Drive/Auto Align/Theta/Kd", 0.0),
              new LoggedTunableNumber("Drive/Auto Align/Theta/Tolerance", 0.0),
              new LoggedTunableNumber("Drive/Auto Align/Theta/Max Velocity", 0.0)),
          new LoggedTunableNumber("Drive/Auto Align/Position Threshold Meters", 0.0));
  public static final double ODOMETRY_FREQUENCY = 250.0;
  public static final double DRIVER_DEADBAND = 0.1;
  public static final double OPERATOR_DEADBAND = 0.1;

  public static final SwerveDriveConstants DRIVE_CONSTANTS =
      new SwerveDriveConstants(
          DRIVE_CONFIG,
          GAINS,
          AUTO_GAINS,
          AUTO_ALIGN_NEAR_CONSTANTS,
          OPERATOR_DEADBAND,
          ODOMETRY_FREQUENCY,
          DRIVER_DEADBAND);

  public static final LimelightConfig LIMELIGHT_CONFIG =
      LimelightConfig.builder()
          .key("center")
          .cameraType(CameraType.LIMELIGHT_3G)
          .horizontalFOV(82.0)
          .verticalFOV(56.2)
          .megatagXYStdev(0.1)
          .metatagThetaStdev(0.0015)
          .megatag2XYStdev(0.001)
          .robotToCameraTransform(new Transform3d(0.0, 0.0, 0.0, new Rotation3d(0.0, 0.0, 0.0)))
          .build();
}
