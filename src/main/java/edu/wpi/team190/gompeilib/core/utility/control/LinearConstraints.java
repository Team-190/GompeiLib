package edu.wpi.team190.gompeilib.core.utility.control;

import edu.wpi.first.units.*;
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
      Measure<DistanceUnit> goalTolerance,
      Measure<LinearVelocityUnit> maxVelocity,
      Measure<LinearAccelerationUnit> maxAcceleration) {
    this(
        new LoggedTunableMeasure<>(String.format("%s/Goal Tolerance", prefix), goalTolerance),
        new LoggedTunableMeasure<>(String.format("%s/Max Velocity", prefix), maxVelocity),
        new LoggedTunableMeasure<>(String.format("%s/Max Acceleration", prefix), maxAcceleration));
  }
}
