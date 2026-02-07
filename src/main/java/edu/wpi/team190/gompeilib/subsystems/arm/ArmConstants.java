package edu.wpi.team190.gompeilib.subsystems.arm;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;
import lombok.Builder;

@Builder
public class ArmConstants {
  public final int ARM_CAN_ID;
  @Builder.Default public final CANBus CAN_LOOP = new CANBus();
  public final ArmParameters ARM_PARAMETERS;
  public final Gains SLOT0_GAINS;
  @Builder.Default public final Gains SLOT1_GAINS = new Gains("Arm/Slot1");
  @Builder.Default public final Gains SLOT2_GAINS = new Gains("Arm/Slot2");

  public final Constraints CONSTRAINTS;

  public final CurrentLimits CURRENT_LIMITS;

  @Builder.Default public final boolean ENABLE_FOC = false;

  @Builder
  public record ArmParameters(
      DCMotor MOTOR_CONFIG,
      Rotation2d MIN_ANGLE,
      Rotation2d MAX_ANGLE,
      Boolean CONTINUOUS_INPUT,
      int NUM_MOTORS,
      double GEAR_RATIO,
      double LENGTH_METERS,
      double MOMENT_OF_INERTIA) {
    static class ArmParametersBuilder {
      ArmParametersBuilder() {
        CONTINUOUS_INPUT = false;
      }
    }
  }

  static {
    ArmParameters p = ArmParameters.builder().build();
    System.out.println(p.CONTINUOUS_INPUT());
  }

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

  @Builder
  public record CurrentLimits(
      double ARM_SUPPLY_CURRENT_LIMIT,
      double ARM_STATOR_CURRENT_LIMIT,
      double ARM_TORQUE_CURRENT_LIMIT) {}

  @Builder(setterPrefix = "with")
  public record Constraints(
      LoggedTunableNumber maxAccelerationRadiansPerSecondSquared,
      LoggedTunableNumber cruisingVelocityRadiansPerSecond,
      LoggedTunableNumber
          goalToleranceRadians) {} // Units intentionally apply to arm rotations, not motor
  // rotations
}
