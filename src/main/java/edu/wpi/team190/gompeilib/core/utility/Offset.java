package edu.wpi.team190.gompeilib.core.utility;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import lombok.Getter;

public class Offset<U extends Unit> {
  @Getter private Measure<U> offset;
  private final Measure<U> step;
  public Measure<U> min;
  public Measure<U> max;

  public Offset(Measure<U> step, Measure<U> min, Measure<U> max) {
    this.offset = step.minus(step);
    this.step = step;
    this.min = min;
    this.max = max;
  }

  public Offset(Measure<U> step) {
    this.offset = step.minus(step);
    this.step = step;
    this.min = step.times(Double.NEGATIVE_INFINITY);
    this.max = step.times(Double.POSITIVE_INFINITY);
  }

  public Measure<U> apply(Measure<U> setpoint) {
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
