package edu.wpi.team190.gompeilib.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import java.util.concurrent.locks.ReentrantLock;

import java.util.concurrent.locks.ReentrantLock;

public class ElevatorConstants {
    public final ReentrantLock lock;

    public final int ELEVATOR_CAN_ID;
    public final double ELEVATOR_GEAR_RATIO;
    public final double DRUM_RADIUS;

    public final double ELEVATOR_SUPPLY_CURRENT_LIMIT;
    public final double ELEVATOR_STATOR_CURRENT_LIMIT;

    public final ElevatorParameters ELEVATOR_PARAMETERS;
    public final Gains SLOT0_GAINS;
    public final Gains SLOT1_GAINS;
    public final Gains SLOT3_GAINS;
    public final Constraints CONSTRAINTS;

    public ElevatorConstants(int ELEVATOR_CAN_ID, double ELEVATOR_GEAR_RATIO, double DRUM_RADIUS, double ELEVATOR_SUPPLY_CURRENT_LIMIT, double ELEVATOR_STATOR_CURRENT_LIMIT, ElevatorParameters ELEVATOR_PARAMETERS, Gains SLOT0_GAINS, Gains SLOT1_GAINS, Gains SLOT2_GAINS, Constraints CONSTRAINTS) {

    }

    public record Gains(
            LoggedTunableNumber kP,
            LoggedTunableNumber kD,
            LoggedTunableNumber kS,
            LoggedTunableNumber kG,
            LoggedTunableNumber kV,
            LoggedTunableNumber kA) {}

    public record Constraints(
            LoggedTunableNumber maxAccelerationMetersPerSecondSquared,
            LoggedTunableNumber cruisingVelocityMetersPerSecond,
            LoggedTunableNumber goalToleranceMeters) {}

    public record ElevatorParameters(
            DCMotor ELEVATOR_MOTOR_CONFIG,
            double CARRIAGE_MASS_KG,
            double MIN_HEIGHT_METERS,
            double MAX_HEIGHT_METERS,
            int NUM_MOTORS) {}


}
