package edu.wpi.team190.gompeilib.subsystems.arm;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.control.AngularConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import lombok.Builder;
import lombok.NonNull;

@Builder(setterPrefix = "with")
public class ArmConstants {
  @NonNull public final Integer armCANID;
  @NonNull public final CANBus canBus;
  @NonNull public final ArmParameters armParameters;
  @NonNull public final Gains slot0Gains;
  @Builder.Default public final Gains slot1Gains = Gains.builder().build();
  @Builder.Default public final Gains slot2Gains = Gains.builder().build();
  @NonNull public final AngularConstraints constraints;
  @NonNull public final CurrentLimits currentLimits;
  @NonNull public final Boolean enableFOC;
  @NonNull public final InvertedValue invertedValue;

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
}
