package edu.wpi.team190.gompeilib.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;

import java.util.concurrent.locks.ReentrantLock;
import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorConstants {
  public final int ELEVATOR_CAN_ID;
  public final boolean ON_CANIVORE;
  public final double ELEVATOR_GEAR_RATIO;
  public final double DRUM_RADIUS;

  public final double ELEVATOR_SUPPLY_CURRENT_LIMIT;
  public final double ELEVATOR_STATOR_CURRENT_LIMIT;

  public final ElevatorParameters ELEVATOR_PARAMETERS;
  public final Gains SLOT0_GAINS;
  public final Gains SLOT1_GAINS;
  public final Gains SLOT2_GAINS;
  public final Constraints CONSTRAINTS;
  
  public final int[] CLOCKWISE_CAN_IDS;
  public final int[] COUNTERCLOCKWISE_CAN_IDS;
    
  public TalonFX talonFX;

  public ElevatorConstants(
      int ELEVATOR_CAN_ID,
      boolean ON_CANIVORE,
      double ELEVATOR_GEAR_RATIO,
      double DRUM_RADIUS,
      double ELEVATOR_SUPPLY_CURRENT_LIMIT,
      double ELEVATOR_STATOR_CURRENT_LIMIT,
      ElevatorParameters ELEVATOR_PARAMETERS,
      Gains SLOT0_GAINS,
      Gains SLOT1_GAINS,
      Gains SLOT2_GAINS,
      Constraints CONSTRAINTS,
      int[] CLOCKWISE_CAN_IDS,
      int[] COUNTERCLOCKWISE_CAN_IDS) {
    
    this.lock = new ReentrantLock();

    this.ELEVATOR_CAN_ID = ELEVATOR_CAN_ID;
    this.ON_CANIVORE = ON_CANIVORE;
    this.ELEVATOR_GEAR_RATIO = ELEVATOR_GEAR_RATIO;
    this.DRUM_RADIUS = DRUM_RADIUS;

    this.ELEVATOR_SUPPLY_CURRENT_LIMIT = ELEVATOR_SUPPLY_CURRENT_LIMIT;
    this.ELEVATOR_STATOR_CURRENT_LIMIT = ELEVATOR_STATOR_CURRENT_LIMIT;

    this.ELEVATOR_PARAMETERS = ELEVATOR_PARAMETERS;
    this.SLOT0_GAINS = SLOT0_GAINS;
    this.SLOT1_GAINS = SLOT1_GAINS;
    this.SLOT2_GAINS = SLOT2_GAINS;
    this.CONSTRAINTS = CONSTRAINTS;

    this.COUNTERCLOCKWISE_CAN_IDS = COUNTERCLOCKWISE_CAN_IDS;
    this.CLOCKWISE_CAN_IDS = CLOCKWISE_CAN_IDS;
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
