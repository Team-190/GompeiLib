package edu.wpi.team190.gompeilib.subsystems.elevator;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.team190.gompeilib.core.utility.Constraints;
import edu.wpi.team190.gompeilib.core.utility.Gains;
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
  @Builder.Default public final Gains slot1Gains = Gains.builder().withPrefix("").build();
  @Builder.Default public final Gains slot2Gains = Gains.builder().withPrefix("").build();
  @NonNull public final Constraints<DistanceUnit> constraints;

  @Singular(value = "alignedFollowerCANID")
  @NonNull
  public final Set<Integer> alignedFollowerCANIDs;

  @Singular(value = "opposedFollowerCANID")
  @NonNull
  public final Set<Integer> opposedFollowerCANIDs;

  @Builder(setterPrefix = "with")
  public record ElevatorParameters(
      @NonNull DCMotor ELEVATOR_MOTOR_CONFIG,
      @NonNull Double CARRIAGE_MASS_KG,
      @NonNull Double MIN_HEIGHT_METERS,
      @NonNull Double MAX_HEIGHT_METERS,
      @NonNull Integer NUM_MOTORS) {}
}
