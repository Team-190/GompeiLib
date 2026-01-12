package edu.wpi.team190.gompeilib.subsystems.roller;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import static edu.wpi.first.units.Units.*;


import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public static class RollerIOInputs {
    public Rotation2d position = new Rotation2d();
    public AngularVelocity velocity = RadiansPerSecond.zero();
    public double appliedVolts = 0;
    public double supplyCurrentAmps = 0.0;
    public double torqueCurrentAmps = 0.0;
    public Temperature temperatureCelsius = Celsius.zero();
  }

  public default void updateInputs(RollerIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setVelocity(AngularVelocity velocity) {}
}