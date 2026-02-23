package edu.wpi.team190.gompeilib.core.utility.control;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import lombok.Builder;
import lombok.NonNull;

/** Specifically for Angular constraints (Degrees, Radians, Rotations). */
@Builder(setterPrefix = "with")
public record LinearConstraints(
    LoggedTunableMeasure<DistanceUnit> goalTolerance,
    LoggedTunableMeasure<LinearVelocityUnit> maxVelocity,
    LoggedTunableMeasure<LinearAccelerationUnit> maxAcceleration) {

  @Builder(setterPrefix = "with")
  public LinearConstraints(
      @NonNull String prefix,
      @NonNull Angle baseUnit,
      Measure<DistanceUnit> goalTolerance,
      Measure<LinearVelocityUnit> maxVelocity,
      Measure<LinearAccelerationUnit> maxAcceleration) {
    this(
        new LoggedTunableMeasure<>(
            String.format("%s/Goal Tolerance (%s)", prefix, baseUnit), goalTolerance),
        new LoggedTunableMeasure<>(
            String.format("%s/Max Velocity (%s/s)", prefix, baseUnit), maxVelocity),
        new LoggedTunableMeasure<>(
            String.format("%s/Max Acceleration (%s/s^2)", prefix, baseUnit), maxAcceleration));
  }
}
