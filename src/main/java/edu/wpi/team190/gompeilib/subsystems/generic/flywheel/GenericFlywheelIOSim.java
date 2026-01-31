package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelIO.GenericFlywheelIOInputs;
import java.util.Arrays;

public class GenericFlywheelIOSim {

  // private DCMotorSim motorSim1;
  // private DCMotorSim motorSim2;

  private FlywheelSim sim;

  private final ProfiledPIDController feedback;
  private SimpleMotorFeedforward feedforward;
  private final int numMotors;

  private double appliedVolts;
  private boolean isClosedLoop;
  private double integratedAngularPosition;

  public GenericFlywheelIOSim(GenericFlywheelConstants constants) {
    sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                constants.MOTOR_CONFIGS[0], constants.MOMENT_OF_INERTIA, constants.GEAR_RATIO),
            constants.MOTOR_CONFIGS[0]);
    feedback =
        new ProfiledPIDController(
            constants.GAINS.kP().get(),
            0,
            constants.GAINS.kD().get(),
            new TrapezoidProfile.Constraints(
                constants.CONSTRAINTS.cruisingVelocityRadiansPerSecond().get(),
                constants.CONSTRAINTS.maxAccelerationRadiansPerSecondSquared().get()));
    feedforward =
        new SimpleMotorFeedforward(
            constants.GAINS.kS().get(), constants.GAINS.kV().get(), constants.GAINS.kA().get());

    appliedVolts = 0;
    isClosedLoop = true;

    numMotors = constants.NUM_MOTORS;
  }

  public void updateInputs(GenericFlywheelIOInputs inputs) {
    integratedAngularPosition += sim.getAngularVelocityRadPerSec() * GompeiLib.getLoopPeriod();
    if (isClosedLoop) {
      appliedVolts =
          feedback.calculate(integratedAngularPosition)
              + feedforward.calculate(feedback.getSetpoint().velocity);
    }

    appliedVolts = MathUtil.clamp(appliedVolts, -12, 12);
    sim.setInputVoltage(appliedVolts);
    sim.update(GompeiLib.getLoopPeriod()); // needs to be a constant

    inputs.positionRadians = new Rotation2d(integratedAngularPosition);
    inputs.velocityErrorRadiansPerSecond = feedback.getPositionError();
    inputs.velocitySetpointRadiansPerSecond = feedback.getSetpoint().velocity;

    Arrays.fill(inputs.appliedVolts, appliedVolts);
    Arrays.fill(inputs.currentsAmps, sim.getCurrentDrawAmps() / numMotors);
    Arrays.fill(inputs.temperaturesCelsius, 0);

    inputs.velocityGoalRadiansPerSecond = feedback.getGoal().velocity;
  }

  public void setFlywheelVoltage(double volts) {
    isClosedLoop = false;
    appliedVolts = volts;
  }

  public void setVelocity(double velocityRadiansPerSecond) {
    isClosedLoop = true;
    feedback.setGoal(velocityRadiansPerSecond);
  }

  public void updateGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    feedback.setPID(kP, 0, kD);
    feedforward = new SimpleMotorFeedforward(kS, kG, kV, kA);
  }

  public void updateConstraints(double maxAcceleration, double cruisingVelocity) {
    feedback.setConstraints(new TrapezoidProfile.Constraints(cruisingVelocity, maxAcceleration));
  }
}
