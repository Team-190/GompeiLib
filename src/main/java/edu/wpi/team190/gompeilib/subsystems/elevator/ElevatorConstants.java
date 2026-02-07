package edu.wpi.team190.gompeilib.subsystems.elevator;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import java.util.Set;
import lombok.Builder;
import lombok.NonNull;
import lombok.Singular;

@Builder(setterPrefix = "with")
public class ElevatorConstants {
  @NonNull public final Integer leaderCANID;
  @NonNull public final CANBus canBus = new CANBus();
  @NonNull public final Double elevatorGearRatio;
  @NonNull public final Double drumRadius;

  @NonNull public final Double elevatorSupplyCurrentLimit;
  @NonNull public final Double elevatorStatorCurrentLimit;

  @NonNull public final ElevatorParameters elevatorParameters;
  @NonNull public final Gains slot0Gains;
  @NonNull public final Gains slot1Gains = new Gains("Elevator/Slot1");
  @NonNull public final Gains slot2Gains = new Gains("Elevator/Slot2");
  @NonNull public final Constraints constraints;

  @Singular(value = "alignedFollowerCANIDs")
  @NonNull
  public final Set<Integer> alignedFollowerCANIDs;

  @Singular(value = "opposedFollowerCANIDs")
  @NonNull
  public final Set<Integer> opposedFollowerCANIDs;

  @Builder(setterPrefix = "with")
  public record Gains(
      @NonNull LoggedTunableNumber kP,
      @NonNull LoggedTunableNumber kD,
      @NonNull LoggedTunableNumber kS,
      @NonNull LoggedTunableNumber kG,
      @NonNull LoggedTunableNumber kV,
      @NonNull LoggedTunableNumber kA) {
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
      @NonNull LoggedTunableNumber maxAccelerationMetersPerSecondSquared,
      @NonNull LoggedTunableNumber cruisingVelocityMetersPerSecond,
      @NonNull LoggedTunableNumber goalToleranceMeters) {}

  @Builder(setterPrefix = "with")
  public record ElevatorParameters(
      @NonNull DCMotor ELEVATOR_MOTOR_CONFIG,
      @NonNull Double CARRIAGE_MASS_KG,
      @NonNull Double MIN_HEIGHT_METERS,
      @NonNull Double MAX_HEIGHT_METERS,
      @NonNull Integer NUM_MOTORS) {}
}
