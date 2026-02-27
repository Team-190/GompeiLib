package edu.wpi.team190.gompeilib.core.utility;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class Offset<U extends Unit> {
  private final double setpoint;
  private double offset;
  private double step;
  private final double min;
  private final double max;
  private final U unit;

<<<<<<< 0ffsets-feb-22
  /**
   * @param setpoint the base setpoint to be modified
   * @param step the amount to increment/decrement by
   * @param min the minimum value
   * @param max the maximum value
   * @param unit the base unit for the measures to be handled in
   */
  public Offset(Measure<U> setpoint, Measure<U> step, Measure<U> min, Measure<U> max, U unit) {
    this.setpoint = setpoint.baseUnitMagnitude();
    this.offset = 0;
    this.step = step.baseUnitMagnitude();
    this.min = min.baseUnitMagnitude() - this.setpoint;
    this.max = max.baseUnitMagnitude() - this.setpoint;
    this.unit = unit;
  }

  /**
   * @param setpoint the base setpoint to be modified
   * @param step the amount to increment/decrement by
   * @param unit the base unit for the measures to be handled in
   */
  public Offset(Measure<U> setpoint, Measure<U> step, U unit) {
    this.setpoint = setpoint.baseUnitMagnitude();
    this.offset = 0;
    this.step = step.baseUnitMagnitude();
    this.min = Double.NEGATIVE_INFINITY;
    this.max = Double.POSITIVE_INFINITY;
    this.unit = unit;
  }

  /**
   * Returns the offset measurement.
   *
   * @return
   */
  public Measure<?> applyOffset() {
    return unit.of(setpoint + offset);
  }

  /**
   * Increases the offset by one step. If the offset would be out of range, it sets the offset to
   * the maximum value.
   */
  public void increment() {
    if (offset + step > max) {
      offset = max;
    } else {
      offset += step;
    }
  }

  /**
   * Decreases the offset by one step. If the offset would be out of range, it sets the offset to
   * the minimum value.
   */
  public void decrement() {
    if (offset - step < min) {
      offset = min;
    } else {
      offset -= step;
    }
  }

  /**
   * Increases the offset value by a specified step value. If the offset would be out of range, it
   * sets the offset to the maximum value.
   *
   * @param step
   */
  public void increment(Measure<U> step) {
    if (offset + step.baseUnitMagnitude() > max) {
      offset = max;
    } else {
      offset += step.baseUnitMagnitude();
    }
  }

  /**
   * Decreases the offset value by a specified step value. If the offset would be out of range, it
   * sets the offset to the minimum value.
   *
   * @param step
   */
  public void decrement(Measure<U> step) {
    if (offset - step.baseUnitMagnitude() < min) {
      offset = min;
    } else {
      offset -= step.baseUnitMagnitude();
    }
  }

  /** Sets the offset to zero. */
  public void reset() {
    offset = 0;
=======
  private double initialSetpoint;
  @Setter private double offset;

  private double min;
  private double max;

  public Offset(double initialSetpoint, double offset, double min, double max) {
    this.initialSetpoint = initialSetpoint;
    this.offset = offset;
    this.min = min;
    this.max = max;
  }

  public boolean inRange() {
    return initialSetpoint + offset >= min && initialSetpoint + offset <= max;
  }

  public double getSetpoint() {
    return inRange()
        ? initialSetpoint + offset
        : Math.max(min, Math.min(max, initialSetpoint + offset));
>>>>>>> feature-generic-offset
  }
}
