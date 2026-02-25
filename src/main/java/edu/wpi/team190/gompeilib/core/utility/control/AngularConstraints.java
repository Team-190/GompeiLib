package edu.wpi.team190.gompeilib.core.utility.control;

import edu.wpi.first.units.*;
import edu.wpi.team190.gompeilib.core.utility.tunable.LoggedTunableMeasure;
import lombok.Builder;
import lombok.NonNull;

/** Specifically for Angular constraints (Degrees, Radians, Rotations). */
@Builder(setterPrefix = "with")
public record AngularConstraints(
    LoggedTunableMeasure<AngleUnit> goalTolerance,
    LoggedTunableMeasure<AngularVelocityUnit> maxVelocity,
    LoggedTunableMeasure<AngularAccelerationUnit> maxAcceleration) {

  @Builder(setterPrefix = "with", builderMethodName = "fromMeasures")
  public AngularConstraints(
      @NonNull String prefix,
      Measure<AngleUnit> goalTolerance,
      Measure<AngularVelocityUnit> maxVelocity,
      Measure<AngularAccelerationUnit> maxAcceleration) {
    this(
        new LoggedTunableMeasure<>(String.format("%s/Goal Tolerance", prefix), goalTolerance),
        new LoggedTunableMeasure<>(String.format("%s/Max Velocity", prefix), maxVelocity),
        new LoggedTunableMeasure<>(String.format("%s/Max Acceleration", prefix), maxAcceleration));
  }
}
