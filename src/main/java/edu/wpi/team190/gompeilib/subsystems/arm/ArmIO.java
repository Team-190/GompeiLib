package edu.wpi.team190.gompeilib.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.team190.gompeilib.core.utility.GainSlot;

public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs {
        public Rotation2d armPosition = new Rotation2d();

        public double armVelocityRadiansPerSecond = 0.0;
        public double accelerationRadiansPerSecondSquared = 0.0;

        public double[] armAppliedVolts = new double[] {};
        public double[] armSupplyCurrentAmps = new double[] {};
        public double[] armTorqueCurrentAmps = new double[] {};
        public double[] armTemperatureCelsius = new double[] {};

        public Rotation2d armPositionGoal = new Rotation2d();
        public Rotation2d armPositionSetpoint = new Rotation2d();
        public Rotation2d armPositionError = new Rotation2d();
    }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setArmVoltage(double volts) {}

  public void setPosition(Rotation2d position);

  public default void setArmPositionGoal(Rotation2d positionGoal) {}

  public void updateGains(double kP, double kD, double kS, double kV, double kA, double kG);
  
  public void setPositionGoal(double positionMeters, GainSlot slot);
  public void updateGains(
      double kP, double kD, double kS, double kV, double kA, double kG, GainSlot slot);

  public default void updateArmConstraints(double maxAcceleration, double cruisingVelocity) {}

  public default void zeroArmPosition() {}

}
