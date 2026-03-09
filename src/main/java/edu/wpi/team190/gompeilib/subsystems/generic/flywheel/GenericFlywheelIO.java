package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.RadiansPerSecond;

public interface GenericFlywheelIO {

  @AutoLog
  public static class GenericFlywheelIOInputs {
    public Rotation2d position = new Rotation2d();
    public AngularVelocity velocity = RadiansPerSecond.of(0.0);

    public Voltage[] appliedVolts = new Voltage[] {};
    public Current[] supplyCurrentAmps = new Current[] {};
    public Current[] torqueCurrentAmps = new Current[] {};
    public Temperature[] temperatureCelsius = new Temperature[] {};

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

  default boolean atVoltageGoal(Voltage voltageReference) {return false;}

  default boolean atCurrentGoal(Current currentReference) {return false;}

  default boolean atVelocityGoal(AngularVelocity velocityReference) {return false;}

  default void updateGains(double kP, double kI, double kD, double kS, double kV, double kA, double kG, GainSlot gainSlot) {}

  default void updateConstraints(
      AngularAcceleration maxAcceleration,
      AngularVelocity maxVelocity,
      AngularVelocity goalTolerance) {}
}
