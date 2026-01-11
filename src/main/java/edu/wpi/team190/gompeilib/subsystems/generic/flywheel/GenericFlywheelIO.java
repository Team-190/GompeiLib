package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GenericFlywheelIO {

    @AutoLog
    public static class WhiplashShooterIOInputs {
        public Rotation2d positionRadians;
        public double velocityRadiansPerSecond;

        public double[] appliedVolts = new double[] {};
        public double[] currentsAmps = new double[] {};
        public double[] temperaturesCelsius = new double[] {};

        public double velocityGoalRadiansPerSecond;
        public double velocitySetpointRadiansPerSecond;
        public double velocityErrorRadiansPerSecond;
    }

    public default void updateInputs(WhiplashShooterIOInputs inputs) {
    }

    public default void setVoltage(double volts) {
    }

    public default void setVelocity(double[] velocityRadiansPerSecond) {
    }

    public default void setPID(double kp, double ki, double kd) {
    }

    public default void setFeedforward(double ks, double kv, double ka) {
    }

    public default void setProfile(
            double maxAccelerationRadiansPerSecondSquared, double goalToleranceRadiansPerSecond) {
    }

    public default boolean atGoal() {
        return false;
    }

    public default void stop() {
    }
}
