package edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.util.LoggedTunableNumber;
import java.util.concurrent.locks.ReentrantLock;

public class DriveConstants {
  public final ReentrantLock lock;

  public final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FRONT_LEFT;
  public final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FRONT_RIGHT;
  public final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BACK_LEFT;
  public final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BACK_RIGHT;

  public final DriveConfig DRIVE_CONFIG;

  public final Gains GAINS;
  public final AutoAlignGains AUTO_ALIGN_GAINS;
  public final AutoAlignGains AUTO_GAINS;

  public final double ODOMETRY_FREQUENCY;
  public final double DRIVER_DEADBAND;
  public final double OPERATOR_DEADBAND;

  public DriveConstants(
      DriveConfig driveConfig,
      Gains gains,
      AutoAlignGains autoAlignGains,
      AutoAlignGains autoGains,
      double odometryFrequency,
      double driverDeadband,
      double operatorDeadband) {
      this.lock = new ReentrantLock();
    this.FRONT_LEFT = driveConfig.frontLeft();
    this.FRONT_RIGHT = driveConfig.frontRight();
    this.BACK_LEFT = driveConfig.backLeft();
    this.BACK_RIGHT = driveConfig.backRight();
    this.DRIVE_CONFIG = driveConfig;
    this.GAINS = gains;
    this.AUTO_ALIGN_GAINS = autoAlignGains;
    this.AUTO_GAINS = autoGains;
    this.ODOMETRY_FREQUENCY = odometryFrequency;
    this.DRIVER_DEADBAND = driverDeadband;
    this.OPERATOR_DEADBAND = operatorDeadband;
  }

  public record DriveConfig(
      String canBus,
      int pigeon2Id,
      double maxLinearVelocityMetersPerSecond,
      double wheelRadiusMeters,
      DCMotor driveModel,
      DCMotor turnModel,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          frontLeft,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          frontRight,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          backLeft,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          backRight,
      double bumperWidth,
      double bumperLength) {
    public double driveBaseRadius() {
      return Math.hypot(
          (Math.abs(frontLeft.LocationX) + Math.abs(frontRight.LocationX)) / 2.0,
          (Math.abs(frontLeft.LocationY) + Math.abs(backLeft.LocationY)) / 2.0);
    }

    public double maxAngularVelocity() {
      return maxLinearVelocityMetersPerSecond / driveBaseRadius();
    }

    public Translation2d[] getModuleTranslations() {
      return new Translation2d[] {
        new Translation2d(frontLeft.LocationX, frontLeft.LocationY),
        new Translation2d(frontRight.LocationX, frontRight.LocationY),
        new Translation2d(backLeft.LocationX, backLeft.LocationY),
        new Translation2d(backRight.LocationX, backRight.LocationY)
      };
    }

    public SwerveDriveKinematics kinematics() {
      return new SwerveDriveKinematics(getModuleTranslations());
    }
  }

  public record Gains(
      LoggedTunableNumber drive_Ks,
      LoggedTunableNumber drive_Kv,
      LoggedTunableNumber drive_Kp,
      LoggedTunableNumber drive_Kd,
      LoggedTunableNumber turn_Kp,
      LoggedTunableNumber turn_Kd) {}

  public record AutoAlignGains(
      LoggedTunableNumber translation_Kp,
      LoggedTunableNumber translation_Kd,
      LoggedTunableNumber rotation_Kp,
      LoggedTunableNumber rotation_Kd) {}

  public record PIDControllerConstants(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber tolerance,
      LoggedTunableNumber maxVelocity) {}

  public static record AlignRobotToAprilTagConstants(
      PIDControllerConstants xPIDConstants,
      PIDControllerConstants yPIDConstants,
      PIDControllerConstants omegaPIDConstants,
      LoggedTunableNumber positionThresholdMeters) {}
}
