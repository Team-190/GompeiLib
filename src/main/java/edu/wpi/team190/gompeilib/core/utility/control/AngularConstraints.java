package edu.wpi.team190.gompeilib.core.utility.control;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import lombok.Builder;
import lombok.NonNull;

/** Specifically for Angular constraints (Degrees, Radians, Rotations). */
@Builder(setterPrefix = "with")
public record AngularConstraints(
    LoggedTunableMeasure<AngleUnit> goalTolerance,
    LoggedTunableMeasure<AngularVelocityUnit> maxVelocity,
    LoggedTunableMeasure<AngularAccelerationUnit> maxAcceleration) {

  @Builder(setterPrefix = "with")
  public AngularConstraints(
      @NonNull String prefix,
      @NonNull Angle baseUnit,
      Measure<AngleUnit> goalTolerance,
      Measure<AngularVelocityUnit> maxVelocity,
      Measure<AngularAccelerationUnit> maxAcceleration) {
    this(
        new LoggedTunableMeasure<>(
            String.format("%s/Goal Tolerance (%s)", prefix, baseUnit), goalTolerance),
        new LoggedTunableMeasure<>(
            String.format("%s/Max Velocity (%s/s)", prefix, baseUnit), maxVelocity),
        new LoggedTunableMeasure<>(
            String.format("%s/Max Acceleration (%s/s^2)", prefix, baseUnit), maxAcceleration));
  }
}
