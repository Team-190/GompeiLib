package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GenericRollerIO {
  @AutoLog
  public static class GenericRollerIOInputs {
    public Rotation2d positionRadians = new Rotation2d();
    public double velocityRadiansPerSecond = 0.0;

    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] temperatureCelsius = new double[] {};
  }

  public default void updateInputs(GenericRollerIOInputs inputs) {}

  public default void setVoltage(double volts) {}
}
