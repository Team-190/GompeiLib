package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GenericFlywheelIO {

  @AutoLog
  public static class GenericFlywheelIOInputs {
    public Rotation2d positionRadians = new Rotation2d();
    public double velocityRadiansPerSecond = 0.0;

    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] temperatureCelsius = new double[] {};

    public double velocityGoalRadiansPerSecond = 0.0;
    public double velocitySetpointRadiansPerSecond = 0.0;
    public double velocityErrorRadiansPerSecond = 0.0;
  }

  public default void updateInputs(GenericFlywheelIOInputs inputs) {}
  ;

  public default void setVoltage(double volts) {}
  ;

  public default void setAmps(double amps) {}
  ;

  public default void setVelocity(double velocityRadiansPerSecond) {}
  ;

  public default void setVelocityTorque(double velocityRadiansPerSecond) {}
  ;

  public default void setPID(double kP, double kI, double kD) {}
  ;

  public default void setFeedforward(double kS, double kV, double kA) {}
  ;

  public default void setProfile(
      double maxAccelerationRadiansPerSecondSquared,
      double cruisingVelocity,
      double goalToleranceRadiansPerSecond) {}
  ;

  public default double getFlywheelVelocityGoal() {
    return 0;
  }

  public default boolean atGoal() {
    return false;
  }
  ;

  public default void stop() {}
  ;
}
