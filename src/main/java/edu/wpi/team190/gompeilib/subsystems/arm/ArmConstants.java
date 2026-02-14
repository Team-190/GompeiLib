package edu.wpi.team190.gompeilib.subsystems.arm;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import lombok.Builder;
import lombok.NonNull;

@Builder(setterPrefix = "with")
public class ArmConstants {
  @NonNull public final Integer armCANID;
  @NonNull public final CANBus canBus;
  @NonNull public final ArmParameters armParameters;
  @NonNull public final Gains slot0Gains;
  @Builder.Default public final Gains slot1Gains = new Gains("Arm/Slot1");
  @Builder.Default public final Gains slot2Gains = new Gains("Arm/Slot2");
  @NonNull public final Constraints constraints;
  @NonNull public final CurrentLimits currentLimits;
  @NonNull public final Boolean enableFOC;

  @Builder(setterPrefix = "with")
  public record ArmParameters(
      @NonNull DCMotor motorConfig,
      @NonNull Rotation2d minAngle,
      @NonNull Rotation2d maxAngle,
      @NonNull Boolean continuousOutput,
      @NonNull Integer numMotors,
      @NonNull Double gearRatio,
      @NonNull Double lengthMeters,
      @NonNull Double momentOfInertia) {}

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
  public record CurrentLimits(
      @NonNull Double armSupplyCurrentLimit,
      @NonNull Double armStatorCurrentLimit,
      @NonNull Double armTorqueCurrentLimit) {}

  @Builder(setterPrefix = "with")
  public record Constraints(
      @NonNull LoggedTunableNumber maxAccelerationRadiansPerSecondSquared,
      @NonNull LoggedTunableNumber cruisingVelocityRadiansPerSecond,
      @NonNull LoggedTunableNumber goalToleranceRadians) {}
}
