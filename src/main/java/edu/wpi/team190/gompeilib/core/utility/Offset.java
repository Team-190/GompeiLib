package edu.wpi.team190.gompeilib.core.utility;

import lombok.Setter;

public class Offset {

    private double initialSetpoint;
    @Setter
    private double offset;

    private double min;
    private double max;

    public Offset(double initialSetpoint, double offset, double min, double max) {
        this.initialSetpoint = initialSetpoint;
        this.offset = offset;
        this.min = min;
        this.max = max;
    }

    /**
     * Returns whether the calculated setpoint (initialSetpoint + offset) is within
     * the valid range of [min, max]
     * 
     * @return true if the calculated setpoint is within the valid range, false
     *         otherwise
     */
    public boolean inRange() {
        return initialSetpoint + offset >= min && initialSetpoint + offset <= max;
    }

    /**
     * Returns the setpoint to send to motors, taking into account the min and max
     * bounds
     * if the offset is outside of the valid range, returns the closest bound.
     * If the offset is within the valid range, returns the offset from the initial
     * setpoint.
     *
     * @return setpoint to send to motors
     */
    public double getSetpoint() {
        return inRange()
                ? initialSetpoint + offset
                : Math.max(min, Math.min(max, initialSetpoint + offset));
    }
}
