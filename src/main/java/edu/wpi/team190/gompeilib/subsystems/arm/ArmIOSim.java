package edu.wpi.team190.gompeilib.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.GainSlot;
import java.util.Arrays;

public class ArmIOSim implements ArmIO {
  public SingleJointedArmSim armSim;

  private double appliedVolts;

  private final ProfiledPIDController feedback;
  private ArmFeedforward feedforward;

  private boolean isClosedLoop;

  private ArmConstants constants;

  public ArmIOSim(ArmConstants constants) {
    this.constants = constants;

    armSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                constants.armParameters.motorConfig(),
                constants.armParameters.momentOfInertia(),
                constants.armParameters.gearRatio()),
            constants.armParameters.motorConfig(),
            constants.armParameters.gearRatio(),
            constants.armParameters.lengthMeters(),
            constants.armParameters.minAngle().getRadians(),
            constants.armParameters.maxAngle().getRadians(),
            true,
            constants.armParameters.minAngle().getRadians());

    appliedVolts = 0.0;

    feedback =
        new ProfiledPIDController(
            constants.slot0Gains.kP().get(),
            0.0,
            constants.slot0Gains.kD().get(),
            new Constraints(
                constants.constraints.cruisingVelocityRadiansPerSecond().get(),
                constants.constraints.maxAccelerationRadiansPerSecondSquared().get()));
    if (constants.armParameters.continuousOutput()) {
      feedback.enableContinuousInput(
          constants.armParameters.minAngle().getRadians(),
          constants.armParameters.maxAngle().getRadians());
    }
    feedback.setTolerance(constants.constraints.goalToleranceRadians().get());

    feedforward =
        new ArmFeedforward(
            constants.slot0Gains.kS().get(),
            constants.slot0Gains.kV().get(),
            constants.slot0Gains.kA().get(),
            constants.slot0Gains.kG().get());
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (isClosedLoop)
      appliedVolts =
          feedback.calculate(armSim.getAngleRads())
              + feedforward.calculate(
                  feedback.getSetpoint().position, feedback.getSetpoint().velocity);

    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    armSim.setInputVoltage(appliedVolts);
    armSim.update(GompeiLib.getLoopPeriod());

    inputs.position = Rotation2d.fromRadians(armSim.getAngleRads());
    inputs.velocityRadiansPerSecond = armSim.getVelocityRadPerSec();

    inputs.appliedVolts = new double[constants.armParameters.numMotors()];
    inputs.supplyCurrentAmps = new double[constants.armParameters.numMotors()];
    inputs.torqueCurrentAmps = new double[constants.armParameters.numMotors()];
    inputs.temperatureCelsius = new double[constants.armParameters.numMotors()];

    Arrays.fill(inputs.appliedVolts, appliedVolts);
    Arrays.fill(inputs.supplyCurrentAmps, armSim.getCurrentDrawAmps());
    Arrays.fill(inputs.torqueCurrentAmps, armSim.getCurrentDrawAmps());

    inputs.positionGoal = Rotation2d.fromRadians(feedback.getGoal().position);
    inputs.positionSetpoint = Rotation2d.fromRadians(feedback.getSetpoint().position);
    inputs.positionError = Rotation2d.fromRadians(feedback.getPositionError());
  }

  @Override
  public void setVoltage(double appliedVolts) {
    isClosedLoop = false;
    this.appliedVolts = appliedVolts;
  }

  @Override
  public void setSlot(GainSlot slot) {
    switch (slot) {
      case ZERO:
        feedback.setPID(constants.slot0Gains.kP().get(), 0.0, constants.slot0Gains.kD().get());
        break;
      case ONE:
        feedback.setPID(constants.slot1Gains.kP().get(), 0.0, constants.slot1Gains.kD().get());
        break;
      case TWO:
        feedback.setPID(constants.slot2Gains.kP().get(), 0.0, constants.slot2Gains.kD().get());
        break;
    }
  }

  @Override
  public void setPosition(Rotation2d position) {
    armSim.setState(position.getRadians(), 0);
  }

  @Override
  public void setPositionGoal(Rotation2d rotationGoal) {
    isClosedLoop = true;
    feedback.setGoal(rotationGoal.getRadians());
  }

  @Override
  public void updateGains(
      double kP, double kD, double kS, double kV, double kA, double kG, GainSlot slot) {
    feedback.setPID(kP, 0, kD);
    feedforward = new ArmFeedforward(kS, kG, kV);
  }

  @Override
  public void updateConstraints(
      double cruisingVelocityRadiansPerSecond,
      double maxAccelerationRadiansPerSecondSquared,
      double goalToleranceRadians) {
    feedback.setConstraints(
        new Constraints(cruisingVelocityRadiansPerSecond, maxAccelerationRadiansPerSecondSquared));
    feedback.setTolerance(goalToleranceRadians);
  }

  @Override
  public boolean atGoal() {
    return feedback.atGoal();
  }
}
