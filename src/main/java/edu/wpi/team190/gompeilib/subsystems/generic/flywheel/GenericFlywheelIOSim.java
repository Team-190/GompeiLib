package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.team190.gompeilib.core.utility.LinearProfile;
import java.util.Arrays;

public class GenericFlywheelIOSim implements GenericFlywheelIO {

  private final FlywheelSim motorSim;

  private final PIDController feedback;
  private final LinearProfile profile;
  private SimpleMotorFeedforward feedforward;

  private double appliedVolts;

  public GenericFlywheelIOSim(GenericFlywheelConstants constants) {
    motorSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                constants.GENERIC_FLYWHEEL_PARAMETERS.MOTOR_CONFIG(),
                constants.MOMENT_OF_INERTIA,
                constants.GEAR_RATIO),
            constants.GENERIC_FLYWHEEL_PARAMETERS.MOTOR_CONFIG());

    feedback = new PIDController(constants.GAINS.kP().get(), 0.0, constants.GAINS.kD().get());
    feedback.setTolerance(constants.CONSTRAINTS.goalToleranceRadiansPerSecond().get());
    profile =
        new LinearProfile(
            constants.CONSTRAINTS.maxAccelerationRadiansPerSecondSquared().get(),
            0.02); // Loop cycle
    feedforward =
        new SimpleMotorFeedforward(constants.GAINS.kS().get(), constants.GAINS.kV().get());

    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(GenericFlywheelIOInputs inputs) {
    motorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    motorSim.update(0.02); // Loop Period

    inputs.positionRadians =
        Rotation2d.fromRadians(
            motorSim.getAngularVelocityRadPerSec() * 0.02); // Riemann Sum with Loop Period
    inputs.velocityRadiansPerSecond = motorSim.getAngularVelocityRadPerSec();
    Arrays.fill(inputs.appliedVolts, appliedVolts);
    Arrays.fill(inputs.currentsAmps, motorSim.getCurrentDrawAmps());
    inputs.velocityGoalRadiansPerSecond = profile.getGoal();
    inputs.velocitySetpointRadiansPerSecond = feedback.getSetpoint();
    inputs.velocityErrorRadiansPerSecond = feedback.getError();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }

  @Override
  public void setVelocity(double velocityRadiansPerSecond) {
    profile.setGoal(velocityRadiansPerSecond, motorSim.getAngularVelocityRadPerSec());
    appliedVolts =
        feedback.calculate(motorSim.getAngularVelocityRadPerSec(), profile.calculateSetpoint())
            + feedforward.calculate(feedback.getSetpoint());
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    feedback.setPID(kP, kI, kD);
  }

  @Override
  public void setFeedforward(double kS, double kV, double kA) {
    feedforward = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public void setProfile(
      double maxAccelerationRadiansPerSecondSquared, double goalToleranceRadiansPerSecond) {
    profile.setMaxAcceleration(maxAccelerationRadiansPerSecondSquared);
    feedback.setTolerance(goalToleranceRadiansPerSecond);
  }

  @Override
  public boolean atGoal(GenericFlywheelConstants constants) {
    return (Math.abs(profile.getGoal() - feedback.getSetpoint())
            <= constants.CONSTRAINTS.goalToleranceRadiansPerSecond().get())
        && feedback.atSetpoint();
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }
}
