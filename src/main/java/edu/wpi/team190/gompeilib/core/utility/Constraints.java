package edu.wpi.team190.gompeilib.core.utility;

import edu.wpi.first.units.AccelerationUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.VelocityUnit;
import lombok.Builder;
import lombok.NonNull;

/**
 * U is restricted to types compatible with Distance or Angle.
 */
public record Constraints<U extends Unit>(
        LoggedTunableMeasure<U> goalTolerance,
        LoggedTunableMeasure<VelocityUnit<U>> maxVelocity,
        LoggedTunableMeasure<AccelerationUnit<U>> maxAcceleration) {

  @Builder(setterPrefix = "with")
  public Constraints(
          @NonNull String prefix,
          @NonNull U baseUnit,
          Measure<U> goalTolerance,
          Measure<VelocityUnit<U>> maxVelocity,
          Measure<AccelerationUnit<U>> maxAcceleration) {
    this(
            new LoggedTunableMeasure<>(
                    String.format("%s/Goal Tolerance (%s)", prefix, baseUnit.symbol()), goalTolerance),
            new LoggedTunableMeasure<>(
                    String.format("%s/Max Velocity (%s/s)", prefix, baseUnit.symbol()), maxVelocity),
            new LoggedTunableMeasure<>(
                    String.format("%s/Max Acceleration (%s/s^2)", prefix, baseUnit.symbol()),
                    maxAcceleration));
  }
}