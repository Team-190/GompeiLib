package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;

public class GenericFlywheelConstants {

    public final int[] CAN_IDS;

    public final double CURRENT_LIMIT;
    public final double MOMENT_OF_INERTIA;

    public final DCMotor[] MOTOR_CONFIGS;

    public final Gains GAINS;
    public final Constraints CONSTRAINTS;

    public GenericFlywheelConstants(int[] CAN_IDS, double CURRENT_LIMIT, double MOMENT_OF_INERTIA, DCMotor[] MOTOR_CONFIGS, Gains GAINS, Constraints CONSTRAINTS) {
        this.CAN_IDS = CAN_IDS;
        this.CURRENT_LIMIT = CURRENT_LIMIT;
        this.MOMENT_OF_INERTIA = MOMENT_OF_INERTIA;
        this.MOTOR_CONFIGS = MOTOR_CONFIGS;
        this.GAINS = GAINS;
        this.CONSTRAINTS = CONSTRAINTS;
    }

    public record Gains(
            LoggedTunableNumber kP,
            LoggedTunableNumber kD,
            LoggedTunableNumber kS,
            LoggedTunableNumber kG,
            LoggedTunableNumber kV,
            LoggedTunableNumber kA) {
    }

    public record Constraints(
            LoggedTunableNumber maxAccelerationMetersPerSecondSquared,
            LoggedTunableNumber cruisingVelocityMetersPerSecond,
            LoggedTunableNumber goalToleranceMeters) {
    }
}
