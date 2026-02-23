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

    public boolean inRange() {
        return initialSetpoint + offset >= min && initialSetpoint + offset <= max;
    }

    public double getSetpoint() {
        return inRange()
                ? initialSetpoint + offset
                : Math.max(min, Math.min(max, initialSetpoint + offset));
    }
}
