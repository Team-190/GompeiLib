package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import java.util.Set;
import lombok.Builder;
import lombok.NonNull;
import lombok.Singular;

@Builder(setterPrefix = "with")
public class GenericFlywheelConstants {

  @NonNull public final Integer leaderCANID;
  @NonNull public final InvertedValue leaderInversion;

  @NonNull public final CANBus canBus = new CANBus();
  @NonNull public final Boolean enableFOC = false;

  @NonNull public final CurrentLimits currentLimit;
  @NonNull public final Double momentOfInertia;
  @NonNull public final Double gearRatio;

  @NonNull public final DCMotor motorConfig;

  @NonNull public final Gains gains;
  @NonNull public final Constraints constraints;

  @Singular(value = "alignedFollowerCANID")
  @NonNull
  public final Set<Integer> alignedFollowerCANIDs;

  @Singular(value = "opposedFollowerCANID")
  @NonNull
  public final Set<Integer> opposedFollowerCANIDs;

  @Builder(setterPrefix = "with")
  @NonNull
  public record Gains(
      @NonNull LoggedTunableNumber kP,
      @NonNull LoggedTunableNumber kD,
      @NonNull LoggedTunableNumber kS,
      @NonNull LoggedTunableNumber kV,
      @NonNull LoggedTunableNumber kA) {}

  @Builder(setterPrefix = "with")
  public record Constraints(
      @NonNull LoggedTunableNumber maxAccelerationRadiansPerSecondSquared,
      @NonNull LoggedTunableNumber cruisingVelocityRadiansPerSecond,
      @NonNull LoggedTunableNumber goalToleranceRadiansPerSecond) {}

  @Builder(setterPrefix = "with")
  public record CurrentLimits(
      @NonNull Double SUPPLY_CURRENT_LIMIT, @NonNull Double STATOR_CURRENT_LIMIT) {}
}
