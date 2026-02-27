package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.control.LinearProfile;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import java.util.Arrays;

public class GenericFlywheelIOSim implements GenericFlywheelIO {

  private final FlywheelSim motorSim;

  private FlywheelSim sim;

  private final PIDController feedback;
  private SimpleMotorFeedforward feedforward;
  private final LinearProfile profile;

  private double appliedVolts;

  GenericFlywheelConstants constants;

  public GenericFlywheelIOSim(GenericFlywheelConstants constants) {
    motorSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                constants.motorConfig, constants.momentOfInertia, constants.gearRatio),
            constants.motorConfig);

    feedback =
        new PIDController(
            constants.voltageGains.kP().get(), 0.0, constants.voltageGains.kD().get());
    feedback.setTolerance(constants.constraints.goalTolerance().get().in(Radians));
    profile =
        new LinearProfile(
            constants.constraints.maxAcceleration().get().in(RadiansPerSecondPerSecond),
            constants.constraints.maxVelocity().get().in(RadiansPerSecond),
            1 / GompeiLib.getLoopPeriod());
    feedforward =
        new SimpleMotorFeedforward(
            constants.voltageGains.kS().get(), constants.voltageGains.kV().get());

    appliedVolts = 0.0;

    this.constants = constants;
  }

  @Override
  public void updateInputs(GenericFlywheelIOInputs inputs) {
    motorSim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    motorSim.update(1 / GompeiLib.getLoopPeriod());

    inputs.positionRadians =
        Rotation2d.fromRadians(
            motorSim.getAngularVelocityRadPerSec() * 1 / GompeiLib.getLoopPeriod());
    inputs.velocityRadiansPerSecond = motorSim.getAngularVelocityRadPerSec();
    Arrays.fill(inputs.appliedVolts, appliedVolts);
    Arrays.fill(inputs.supplyCurrentAmps, motorSim.getCurrentDrawAmps());
    Arrays.fill(inputs.torqueCurrentAmps, motorSim.getCurrentDrawAmps());
    inputs.velocityGoalRadiansPerSecond = profile.getGoal();
    inputs.velocitySetpointRadiansPerSecond = feedback.getSetpoint();
    inputs.velocityErrorRadiansPerSecond = feedback.getError();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
  }

  @Override
  public void setVelocityVoltage(double velocityRadiansPerSecond) {
    profile.setGoal(velocityRadiansPerSecond, motorSim.getAngularVelocityRadPerSec());
    appliedVolts =
        feedback.calculate(motorSim.getAngularVelocityRadPerSec(), profile.calculateSetpoint())
            + feedforward.calculate(feedback.getSetpoint());
  }

  @Override
  public void setAmps(double amps) {
    double omega = motorSim.getAngularVelocityRadPerSec();

    var motor = motorSim.getGearbox();

    double Kv = motor.KvRadPerSecPerVolt;
    double R = motor.rOhms;

    appliedVolts = omega / Kv + amps * R;
  }

  @Override
  public void setVelocityTorque(double velocityRadiansPerSecond, double feedForward) {
    // Motion profile stays the same
    profile.setGoal(velocityRadiansPerSecond, motorSim.getAngularVelocityRadPerSec());

    double omega = motorSim.getAngularVelocityRadPerSec();
    double setpoint = profile.calculateSetpoint();

    // Velocity PID → amps (torque)
    double amps =
        feedback.calculate(omega, setpoint) + feedForward / motorSim.getGearbox().KtNMPerAmp;

    // Optional current limit
    amps =
        MathUtil.clamp(
            amps,
            -constants.currentLimit.supplyCurrentLimit().in(Amps),
            constants.currentLimit.supplyCurrentLimit().in(Amps));

    // Amps → volts using motor physics
    var motor = motorSim.getGearbox();

    double Kv = motor.KvRadPerSecPerVolt;
    double R = motor.rOhms;

    appliedVolts = omega / Kv + amps * R;
  }

  @Override
  public void setPID(GainSlot slot, double kP, double kI, double kD) {
    feedback.setPID(kP, kI, kD);
  }

  @Override
  public void setFeedforward(GainSlot slot, double kS, double kV, double kA) {
    feedforward = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public void setProfile(
      double maxAccelerationRadiansPerSecondSquared,
      double cruisingVelocityRadiansPerSecond,
      double goalToleranceRadiansPerSecond) {
    profile.setMaxAcceleration(maxAccelerationRadiansPerSecondSquared);
    profile.setMaxVelocity(cruisingVelocityRadiansPerSecond);
    feedback.setTolerance(goalToleranceRadiansPerSecond);
  }

  @Override
  public boolean atGoal() {
    return feedback.atSetpoint();
  }

  @Override
  public void stop() {
    appliedVolts = 0.0;
  }
}
