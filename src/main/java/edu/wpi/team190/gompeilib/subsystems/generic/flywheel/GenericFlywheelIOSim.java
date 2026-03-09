package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.control.LinearProfile;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import java.util.Arrays;

public class GenericFlywheelIOSim implements GenericFlywheelIO {

  private final FlywheelSim motorSim;

  private Voltage appliedVolts;
  private boolean isClosedLoop;

  private final PIDController feedback;
  private SimpleMotorFeedforward feedforward;
  private final LinearProfile profile;

  GenericFlywheelConstants constants;

  private Angle accumulatedPosition;

  public GenericFlywheelIOSim(GenericFlywheelConstants constants) {
    motorSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                constants.motorConfig, constants.momentOfInertia, constants.gearRatio),
            constants.motorConfig);

    appliedVolts = Volts.of(0.0);
    isClosedLoop = false;

    feedback =
        new PIDController(
            constants.voltageGains.kP().get(), 0.0, constants.voltageGains.kD().get());
    feedback.setTolerance(constants.constraints.goalTolerance().get().in(RadiansPerSecond));
    feedforward =
        new SimpleMotorFeedforward(
            constants.voltageGains.kS().get(), constants.voltageGains.kV().get());
    profile =
            new LinearProfile(
                    constants.constraints.maxAcceleration().get().in(RadiansPerSecondPerSecond),
                    constants.constraints.maxVelocity().get().in(RadiansPerSecond),
                    1 / GompeiLib.getLoopPeriod());

    this.constants = constants;

    accumulatedPosition = Radians.of(0.0);
  }

  @Override
  public void updateInputs(GenericFlywheelIOInputs inputs) {
    if (isClosedLoop)
      appliedVolts = Volts.of(feedback.calculate(motorSim.getAngularVelocityRadPerSec()) + feedforward.calculate(feedback.getSetpoint()));

    appliedVolts = Volts.of(MathUtil.clamp(appliedVolts.in(Volts), -12.0, 12.0));
    motorSim.setInputVoltage(appliedVolts.in(Volts));
    motorSim.update(1.0 / GompeiLib.getLoopPeriod());

    accumulatedPosition = Radians.of(accumulatedPosition.in(Radians) + (motorSim.getAngularVelocityRadPerSec() * (1.0 / GompeiLib.getLoopPeriod())));

    inputs.position =
        Rotation2d.fromRadians(accumulatedPosition.in(Radians));
    inputs.velocity = motorSim.getAngularVelocity();

    Arrays.fill(inputs.appliedVolts, appliedVolts);
    Arrays.fill(inputs.supplyCurrentAmps, Amps.of(motorSim.getCurrentDrawAmps()));
    Arrays.fill(inputs.torqueCurrentAmps, Amps.of(motorSim.getCurrentDrawAmps()));

    inputs.velocityGoal = RadiansPerSecond.of(profile.getGoal());
    inputs.velocitySetpoint = RadiansPerSecond.of(feedback.getSetpoint());
    inputs.velocityError = RadiansPerSecond.of(feedback.getError());
  }

  @Override
  public void setVoltageGoal(Voltage voltageGoal) {
    appliedVolts = voltageGoal;
  }

  @Override
  public void setVelocityGoal(AngularVelocity velocityGoal) {
    profile.setGoal(velocityGoal.in(RadiansPerSecond), motorSim.getAngularVelocityRadPerSec());
    appliedVolts =
        Volts.of(feedback.calculate(motorSim.getAngularVelocityRadPerSec(), profile.calculateSetpoint())
            + feedforward.calculate(feedback.getSetpoint()));
  }

  @Override
  public boolean atVoltageGoal(Voltage voltageReference) {
    return appliedVolts.isNear(voltageReference, Millivolts.of(500));
  }

  @Override
  public boolean atVelocityGoal(AngularVelocity velocityReference) {
    return motorSim.getAngularVelocity().isNear(velocityReference, constants.constraints.goalTolerance().get(RadiansPerSecond));
  }

  @Override
  public void updateGains(double kP, double kI, double kD, double kS, double kV, double kA, double kG, GainSlot gainSlot) {
    feedback.setPID(kP, kI, kD);
    feedforward.setKs(kS);
    feedforward.setKv(kV);
    feedforward.setKa(kA);
  }

  @Override
  public void updateConstraints(
          AngularAcceleration maxAcceleration,
          AngularVelocity maxVelocity,
          AngularVelocity goalTolerance) {
    profile.setMaxAcceleration(maxAcceleration.in(RadiansPerSecondPerSecond));
    profile.setMaxVelocity(maxVelocity.in(RadiansPerSecond));
    feedback.setTolerance(goalTolerance.in(RadiansPerSecond));
  }
}
