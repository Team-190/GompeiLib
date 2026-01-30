package edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.subsystems.drivebases.swervedrive.SwerveDriveConstants;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class SwerveModuleIOTalonFXSim extends SwerveModuleIOTalonFX { 

  private final DCMotorSim steerMotorSim;
  private final DCMotorSim driveMotorSim;

  private TalonFXSimState steerController;
  private TalonFXSimState driveController;
  private CANcoderSimState encoderController;
  private double offset;

  private double motorVoltageDrive;

  private double motorVoltageSteer;

  public SwerveModuleIOTalonFXSim(SwerveDriveConstants driveConstants, SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants) {
    // steerMotorSim = motor.getSimState();
    super(driveConstants, constants);
    driveMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                driveConstants.DRIVE_CONFIG.driveModel(),
                constants.DriveInertia,
                constants.DriveMotorGearRatio),
            driveConstants.DRIVE_CONFIG.driveModel());
    steerMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                driveConstants.DRIVE_CONFIG.turnModel(),
                constants.SteerInertia,
                constants.SteerMotorGearRatio),
            driveConstants.DRIVE_CONFIG.turnModel());
  }

  @Override
  @Trace
  public void updateInputs(ModuleIOInputs inputs) {

    driveController.setSupplyVoltage(RobotController.getBatteryVoltage());
    motorVoltageDrive = driveController.getMotorVoltage();

    driveMotorSim.setInputVoltage(motorVoltageDrive);

    driveMotorSim.update(GompeiLib.getLoopPeriod());

    double rotorPositionRotationsDrive =
        driveMotorSim.getAngularPositionRotations() * driveMotorSim.getGearing();
    double rotorVelocityRotationsPerSecondDrive =
        driveMotorSim.getAngularVelocityRadPerSec() / (Math.PI * 2) * driveMotorSim.getGearing();
    driveController.setRawRotorPosition(rotorPositionRotationsDrive);
    driveController.setRotorVelocity(rotorVelocityRotationsPerSecondDrive);
    


    steerController.setSupplyVoltage(RobotController.getBatteryVoltage());
    motorVoltageSteer = steerController.getMotorVoltage();

    steerMotorSim.setInputVoltage(motorVoltageSteer);

    steerMotorSim.update(GompeiLib.getLoopPeriod());

    double rotorPositionRotationsSteer =
        steerMotorSim.getAngularPositionRotations() * steerMotorSim.getGearing();
    double rotorVelocityRotationsPerSecondSteer =
        steerMotorSim.getAngularVelocityRadPerSec() / (Math.PI * 2) * steerMotorSim.getGearing();
    driveController.setRawRotorPosition(rotorPositionRotationsSteer);
    driveController.setRotorVelocity(rotorVelocityRotationsPerSecondSteer);

    encoderController.setRawPosition(steerMotorSim.getAngularPositionRotations() + offset);

    super.updateInputs(inputs);
  }
}