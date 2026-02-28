package edu.wpi.team190.gompeilib.core.utility;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class Offset<U extends Unit> {
  private final Measure<U> setpoint;
  private Measure<U> offset;
  private Measure<U> step;
  private final Measure<U> min;
  private final Measure<U> max;

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

  public Measure<U> getNewSetpoint() {
    return setpoint.plus(offset);
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
}
