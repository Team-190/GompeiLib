package edu.wpi.team190.gompeilib.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.team190.gompeilib.core.utility.GainSlot;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public Rotation2d position = new Rotation2d();

    public double velocityRadiansPerSecond = 0.0;
    public double accelerationRadiansPerSecondSquared = 0.0;

    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] temperatureCelsius = new double[] {};

    public Rotation2d positionGoal = new Rotation2d();
    public Rotation2d positionSetpoint = new Rotation2d();
    public Rotation2d positionError = new Rotation2d();

    public GainSlot slot;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setSlot(GainSlot slot) {}

  public default void setPosition(Rotation2d position) {}

  public default void setPositionGoal(Rotation2d positionGoal) {}

  public default void updateGains(
      double kP, double kD, double kS, double kV, double kA, double kG, GainSlot slot) {}

  public default void updateConstraints(
      double maxAcceleration, double cruisingVelocity, double goalTolerance) {}

  public default boolean atGoal() {
    return false;
  }
}
