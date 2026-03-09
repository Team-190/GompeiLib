package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.littletonrobotics.junction.AutoLog;

public interface GenericFlywheelIO {

  @AutoLog
  public static class GenericFlywheelIOInputs {
    public Rotation2d position = new Rotation2d();
    public AngularVelocity velocity = RadiansPerSecond.of(0.0);

    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] temperatureCelsius = new double[] {};

    public AngularVelocity velocityGoal = RadiansPerSecond.of(0.0);
    public AngularVelocity velocitySetpoint = RadiansPerSecond.of(0.0);
    public AngularVelocity velocityError = RadiansPerSecond.of(0.0);

    public GainSlot gainSlot;
  }

  default void updateInputs(GenericFlywheelIOInputs inputs) {}

  default void setVoltageGoal(Voltage voltageGoal) {}

  default void setCurrentGoal(Current currentGoal) {}

  default void setVelocityGoal(AngularVelocity velocityGoal) {}

  default void setVelocityGoal(AngularVelocity velocityGoal, Current currentFeedforward) {}

  default boolean atVoltageGoal(Voltage voltageReference) {
    return false;
  }

  default boolean atCurrentGoal(Current currentReference) {
    return false;
  }

  default boolean atVelocityGoal(AngularVelocity velocityReference) {
    return false;
  }

  default void updateGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kG,
      GainSlot gainSlot) {}

  default void updateConstraints(
      AngularAcceleration maxAcceleration,
      AngularVelocity maxVelocity,
      AngularVelocity goalTolerance) {}
}
