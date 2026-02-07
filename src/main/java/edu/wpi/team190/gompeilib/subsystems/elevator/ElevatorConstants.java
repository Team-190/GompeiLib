package edu.wpi.team190.gompeilib.subsystems.elevator;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import java.util.Set;
import lombok.Builder;
import lombok.Singular;

@Builder
public class ElevatorConstants {
  public final int LEADER_CAN_ID;
  @Builder.Default public final CANBus CAN_LOOP = new CANBus();
  public final double ELEVATOR_GEAR_RATIO;
  public final double DRUM_RADIUS;

  public final double ELEVATOR_SUPPLY_CURRENT_LIMIT;
  public final double ELEVATOR_STATOR_CURRENT_LIMIT;

  public final ElevatorParameters ELEVATOR_PARAMETERS;
  public final Gains SLOT0_GAINS;
  @Builder.Default public final Gains SLOT1_GAINS = new Gains("Elevator/Slot1");
  @Builder.Default public final Gains SLOT2_GAINS = new Gains("Elevator/Slot2");
  public final Constraints CONSTRAINTS;

  @Singular(value = "ALIGNED_FOLLOWER_CAN_ID")
  public final Set<Integer> ALIGNED_FOLLOWER_CAN_IDS;

  @Singular(value = "OPPOSED_FOLLOWER_CAN_ID")
  public final Set<Integer> OPPOSED_FOLLOWER_CAN_IDS;

  @Builder
  public record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kG,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA) {
    public Gains(String prefix) {
      this(
          new LoggedTunableNumber(prefix + "/kP"),
          new LoggedTunableNumber(prefix + "/kD"),
          new LoggedTunableNumber(prefix + "/kS"),
          new LoggedTunableNumber(prefix + "/kG"),
          new LoggedTunableNumber(prefix + "/kV"),
          new LoggedTunableNumber(prefix + "/kA"));
    }
  }

  @Builder(setterPrefix = "with")
  public record Constraints(
      LoggedTunableNumber maxAccelerationMetersPerSecondSquared,
      LoggedTunableNumber cruisingVelocityMetersPerSecond,
      LoggedTunableNumber goalToleranceMeters) {}

  @Builder
  public record ElevatorParameters(
      DCMotor ELEVATOR_MOTOR_CONFIG,
      double CARRIAGE_MASS_KG,
      double MIN_HEIGHT_METERS,
      double MAX_HEIGHT_METERS,
      int NUM_MOTORS) {}
}
