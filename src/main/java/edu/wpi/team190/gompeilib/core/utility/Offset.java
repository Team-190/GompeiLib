package edu.wpi.team190.gompeilib.core.utility;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

public class Offset<U extends Unit, M extends Measure<U>> {
    private final double setpoint;
    private double offset;
    private double step;
    private final double min;
    private final double max;
    private final U unit;

    /**
     * @param setpoint the base setpoint to be modified
     * @param step     the amount to increment/decrement by
     * @param min      the minimum value
     * @param max      the maximum value
     * @param unit     the base unit for the measures to be handled in
     */
    public Offset(M setpoint, M step, M min, M max, U unit) {
        this.setpoint = setpoint.baseUnitMagnitude();
        this.offset = 0;
        this.step = step.baseUnitMagnitude();
        this.min = min.baseUnitMagnitude();
        this.max = max.baseUnitMagnitude();
        this.unit = unit;
    }

    /**
     * @param setpoint the base setpoint to be modified
     * @param step     the amount to increment/decrement by
     * @param unit     the base unit for the measures to be handled in
     */
    public Offset(M setpoint, M step, U unit) {
        this.setpoint = setpoint.baseUnitMagnitude();
        this.offset = 0;
        this.step = step.baseUnitMagnitude();
        this.min = Double.NEGATIVE_INFINITY;
        this.max = Double.POSITIVE_INFINITY;
        this.unit = unit;
    }

    public Measure<?> applyOffset() {
        return unit.of(setpoint + offset);
    }

    public void increment() {
        if (offset + step > max) {
            offset = max;
        } else {
            offset += step;
        }
    }

    public void decrement() {
        if (offset - step < min) {
            offset = min;
        } else {
            offset -= step;
        }
    }

    public void increment(M step) {
        if (offset + step.baseUnitMagnitude() > max) {
            offset = max;
        } else {
            offset += step.baseUnitMagnitude();
        }
    }

    public void decrement(M step) {
        if (offset - step.baseUnitMagnitude() < min) {
            offset = min;
        } else {
            offset -= step.baseUnitMagnitude();
        }
    }

    public void reset() {
        offset = 0;
    }
}
