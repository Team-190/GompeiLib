// Copyright 2021-2024 FRC 6328
package edu.wpi.team190.gompeilib.subsystems.swervedrive;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.team190.gompeilib.GompeiLib;
import lombok.Getter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    private final DriveConstants driveConstants;
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs;
  private final Module[] modules;

  private final LinearFilter xFilter;
  private final LinearFilter yFilter;
  private double filteredX;
  private double filteredY;

  private final SwerveDriveKinematics kinematics;
  @Getter private Rotation2d rawGyroRotation;
  private SwerveModulePosition[] lastModulePositions;
  @Getter private ChassisSpeeds measuredChassisSpeeds;

  @Getter private final AutoFactory autoFactory;

  private final Supplier<Pose2d> robotPoseSupplier;

  private final PIDController autoXController;
  private final PIDController autoYController;
  private final PIDController autoHeadingController;

  public Drive(
          DriveConstants driveConstants,
          GyroIO gyroIO,
          ModuleIO flModuleIO,
          ModuleIO frModuleIO,
          ModuleIO blModuleIO,
          ModuleIO brModuleIO,
          Supplier<Pose2d> robotPoseSupplier,
          Consumer<Pose2d> resetPoseConsumer,
          DriveConstants.AutoAlignGains autoFeedbackConstants) {
      this.driveConstants = driveConstants;
    this.gyroIO = gyroIO;
    gyroInputs = new GyroIOInputsAutoLogged();
    modules = new Module[4]; // FL, FR, BL, BR
    modules[0] = new Module(driveConstants, flModuleIO, 0);
    modules[1] = new Module(driveConstants, frModuleIO, 1);
    modules[2] = new Module(driveConstants, blModuleIO, 2);
    modules[3] = new Module(driveConstants, brModuleIO, 3);

    // Start threads (no-op if no signals have been created)
    PhoenixOdometryThread.getInstance(driveConstants).start();

    xFilter = LinearFilter.movingAverage(10);
    yFilter = LinearFilter.movingAverage(10);
    filteredX = 0;
    filteredY = 0;

    kinematics = this.driveConstants.DRIVE_CONFIG.kinematics();
    rawGyroRotation = new Rotation2d();
    lastModulePositions = // For delta tracking
        new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
        };

    autoFactory =
        new AutoFactory(
            robotPoseSupplier,
            resetPoseConsumer,
            this::choreoDrive,
            true,
            this);

    this.robotPoseSupplier = robotPoseSupplier;

    this.autoXController = new PIDController(autoFeedbackConstants.translation_Kp().get(), 0, autoFeedbackConstants.translation_Kd().get());
    this.autoYController = new PIDController(autoFeedbackConstants.translation_Kp().get(), 0, autoFeedbackConstants.translation_Kd().get());
    this.autoHeadingController = new PIDController(autoFeedbackConstants.rotation_Kp().get(), 0, autoFeedbackConstants.rotation_Kd().get());
  }

  public void periodic() {
    driveConstants.lock.lock();

    gyroIO.updateInputs(gyroInputs);

    for (int i = 0; i < 4; i++) {
      modules[i].updateInputs();
    }

    driveConstants.lock.unlock();

    Logger.processInputs("Drive/Gyro", gyroInputs);

    for (int i = 0; i < 4; i++) {
      modules[i].periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }

      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(getModuleStates());
      measuredChassisSpeeds = chassisSpeeds;
      Translation2d rawFieldRelativeVelocity =
          new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
              .rotateBy(getRawGyroRotation());

      filteredX = xFilter.calculate(rawFieldRelativeVelocity.getX());
      filteredY = yFilter.calculate(rawFieldRelativeVelocity.getY());
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds optimizedSpeeds = ChassisSpeeds.discretize(speeds, GompeiLib.getLoopPeriod());
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(optimizedSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, driveConstants.DRIVE_CONFIG.maxLinearVelocityMetersPerSecond());

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", speeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i], new SwerveModuleState());
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /**
   * Runs the drive at the desired velocity and torque.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocityTorque(ChassisSpeeds speeds, List<Vector<N2>> forces) {
    if (forces.size() != 4) {
      throw new IllegalArgumentException("Forces array must have 4 elements");
    }
    // Calculate module setpoints
    ChassisSpeeds optimizedSpeeds = ChassisSpeeds.discretize(speeds, GompeiLib.getLoopPeriod());
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(optimizedSpeeds);
    SwerveModuleState[] setpointTorques = new SwerveModuleState[4];
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, driveConstants.DRIVE_CONFIG.maxLinearVelocityMetersPerSecond());

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      Vector<N2> wheelDirection =
          VecBuilder.fill(setpointStates[i].angle.getCos(), setpointStates[i].angle.getSin());
      setpointTorques[i] =
          new SwerveModuleState(
              forces.get(i).dot(wheelDirection) * driveConstants.FRONT_LEFT.DriveMotorGearRatio,
              setpointStates[i].angle);

      setpointStates[i].optimize(modules[i].getAngle());
      setpointTorques[i].optimize(modules[i].getAngle());

      modules[i].runSetpoint(setpointStates[i], setpointTorques[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    Logger.recordOutput("SwerveStates/TorquesOptimized", setpointTorques);
  }

  /** Runs the drive in a straight line with the specified drive current. */
  public void runCharacterization(double amps) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(amps);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = driveConstants.DRIVE_CONFIG.getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return driveConstants.DRIVE_CONFIG.maxLinearVelocityMetersPerSecond();
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / driveConstants.DRIVE_CONFIG.driveBaseRadius();
  }

  /** Returns the field relative velocity in X and Y. */
  public Translation2d getFieldRelativeVelocity() {
    return new Translation2d(filteredX, filteredY);
  }

  /** Returns the current yaw velocity */
  public double getYawVelocity() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  /** Sets PID gains for modules */
  public void setPIDGains(double drive_Kp, double drive_Kd, double turn_Kp, double turn_Kd) {
    for (var module : modules) {
      module.setPID(drive_Kp, drive_Kd, turn_Kp, turn_Kd);
    }
  }

  /** Sets FF gains for modules */
  public void setFFGains(double kS, double kV) {
    for (var module : modules) {
      module.setFF(kS, kV);
    }
  }

  /** Runs a choreo path from swerve samples */
  public void choreoDrive(SwerveSample sample) {
    double xFF = sample.vx;
    double yFF = sample.vy;
    double rotationFF = sample.omega;

    double xFeedback = autoXController.calculate(robotPoseSupplier.get().getX(), sample.x);
    double yFeedback = autoYController.calculate(robotPoseSupplier.get().getY(), sample.y);
    double rotationFeedback =
        autoHeadingController.calculate(robotPoseSupplier.get().getRotation().getRadians(), sample.heading);

    ChassisSpeeds velocity =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback,
            yFF + yFeedback,
            rotationFF + rotationFeedback,
            Rotation2d.fromRadians(sample.heading));

    runVelocity(velocity);
    Logger.recordOutput("Auto/Setpoint", sample.getPose());
  }
}
