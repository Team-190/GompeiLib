package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

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

  public void updateInputs(GenericFlywheelIOInputs inputs);

  public void setVoltage(double volts);

  public void setVelocity(double velocityRadiansPerSecond);

  public void setPID(double kP, double kI, double kD);

  public void setFeedforward(double kS, double kV, double kA);

  public void setProfile(
      double maxAccelerationRadiansPerSecondSquared, double goalToleranceRadiansPerSecond);

  public boolean atGoal();

  public void stop();
}
