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
    armSim = new SingleJointedArmSim(
        LinearSystemId.createSingleJointedArmSystem(
            constants.ARM_PARAMETERS.MOTOR_CONFIG(),
            constants.MOMENT_OF_INERTIA,
            constants.ARM_PARAMETERS.GEAR_RATIO()),
        constants.ARM_PARAMETERS.MOTOR_CONFIG(),
        constants.ARM_PARAMETERS.GEAR_RATIO(),
        constants.ARM_PARAMETERS.LENGTH_METERS(),
        constants.ARM_PARAMETERS.MIN_ANGLE().getRadians(),
        constants.ARM_PARAMETERS.MAX_ANGLE().getRadians(),
        true,
        constants.ARM_PARAMETERS.MIN_ANGLE().getRadians());

    appliedVolts = 0.0;

    feedback = new ProfiledPIDController(
        constants.SLOT0_GAINS.kP().get(),
        0.0,
        constants.SLOT0_GAINS.kD().get(),
        new Constraints(
            constants.CONSTRAINTS.CRUISING_VELOCITY_ROTATIONS_PER_SECOND().get(),
            constants.CONSTRAINTS.MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED().get()));

    feedforward = new ArmFeedforward(
        constants.SLOT0_GAINS.kS().get(),
        constants.SLOT0_GAINS.kV().get(),
        constants.SLOT0_GAINS.kA().get(),
        constants.SLOT0_GAINS.kG().get());
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (isClosedLoop)
      appliedVolts = feedback.calculate(armSim.getAngleRads())
          + feedforward.calculate(
              feedback.getSetpoint().position, feedback.getSetpoint().velocity);

    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);
    armSim.setInputVoltage(appliedVolts);
    armSim.update(GompeiLib.getLoopPeriod());

    inputs.position = Rotation2d.fromRadians(armSim.getAngleRads());
    inputs.velocityRadiansPerSecond = armSim.getVelocityRadPerSec();

    inputs.appliedVolts = new double[constants.ARM_PARAMETERS.NUM_MOTORS()];
    inputs.supplyCurrentAmps = new double[constants.ARM_PARAMETERS.NUM_MOTORS()];
    inputs.torqueCurrentAmps = new double[constants.ARM_PARAMETERS.NUM_MOTORS()];
    inputs.temperatureCelsius = new double[constants.ARM_PARAMETERS.NUM_MOTORS()];


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
        feedback.setPID(constants.SLOT0_GAINS.kP().get(), 0.0, constants.SLOT0_GAINS.kD().get());
        break;
      case ONE:
        feedback.setPID(constants.SLOT1_GAINS.kP().get(), 0.0, constants.SLOT1_GAINS.kD().get());
        break;
      case TWO:
        feedback.setPID(constants.SLOT2_GAINS.kP().get(), 0.0, constants.SLOT2_GAINS.kD().get());
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
      double cruisingVelocityRadiansPerSecond, double maxAccelerationRadiansPerSecondSquared) {
    feedback.setConstraints(
        new Constraints(cruisingVelocityRadiansPerSecond, maxAccelerationRadiansPerSecondSquared));
  }
}
