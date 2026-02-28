package edu.wpi.team190.gompeilib.core.utility;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import lombok.Getter;

public class Offset<U extends Unit> {
  @Getter private Measure<U> setpoint;
  @Getter private Measure<U> offset;
  private Measure<U> step;
  public Measure<U> min;
  public Measure<U> max;

  public Offset(Measure<U> setpoint, Measure<U> step, Measure<U> min, Measure<U> max) {
    this.setpoint = setpoint;
    this.offset = setpoint.times(0);
    this.step = step;
    this.min = min.minus(setpoint);
    this.max = max.minus(setpoint);
  }

  public Offset(Measure<U> setpoint, Measure<U> step) {
    this.setpoint = setpoint;
    this.offset = setpoint.times(0);
    this.step = step;
    this.min = step.times(Double.NEGATIVE_INFINITY);
    this.max = step.times(Double.POSITIVE_INFINITY);
  }

  public void setSetpoint(Measure<U> setpoint) {
    this.min = min.plus(this.setpoint).minus(setpoint);
    this.max = max.plus(this.setpoint).minus(setpoint);
    this.setpoint = setpoint;
    offset = clamp(offset);
  }

  public Measure<U> getNewSetpoint() {
    double sign = Math.signum(setpoint.magnitude());

    // Apply offset in the direction of the setpoint's sign
    // positive offset always increases magnitude, negative always decreases
    var newMagnitude = setpoint.plus(offset.times(sign));

    // Clamp to zero if we've crossed zero (don't flip sign)
    if (Math.signum(newMagnitude.magnitude()) != sign && sign != 0) {
      return setpoint.times(0); // return zero in same unit
    }

    return newMagnitude;
  }

  public void increment() {
    Measure<U> next = offset.plus(step);
    if (next.gt(max)) {
      offset = max;
    } else {
      offset = next;
    }
  }

  public void decrement() {
    Measure<U> next = offset.minus(step);
    if (next.lt(min)) {
      offset = min;
    } else {
      offset = next;
    }
  }

  public void increment(Measure<U> step) {
    Measure<U> next = offset.plus(step);
    if (next.gt(max)) {
      offset = max;
    } else {
      offset = next;
    }
  }

  public void decrement(Measure<U> step) {
    Measure<U> next = offset.minus(step);
    if (next.lt(min)) {
      offset = min;
    } else {
      offset = next;
    }
  }

  public void reset() {
    offset = offset.times(0);
  }

  public Measure<U> clamp(Measure<U> value) {
    if (value.gt(max)) {
      return max;
    } else if (value.lt(min)) {
      return min;
    } else {
      return value;
    }
  }
}
