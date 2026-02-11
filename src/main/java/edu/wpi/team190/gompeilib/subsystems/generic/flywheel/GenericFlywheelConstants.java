package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.control.AngularConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.CurrentLimits;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import java.util.Set;
import lombok.Builder;
import lombok.NonNull;
import lombok.Singular;

@Builder(setterPrefix = "with")
public class GenericFlywheelConstants {

  @NonNull public final Integer leaderCANID;
  @NonNull public final InvertedValue leaderInversion;

  @NonNull public final CANBus canBus;
  @NonNull public final Boolean enableFOC;

  @NonNull public final CurrentLimits currentLimit;
  @NonNull public final Double momentOfInertia;
  @NonNull public final Double gearRatio;

  @NonNull public final DCMotor motorConfig;

  @NonNull public final Gains gains;
  @NonNull public final AngularConstraints constraints;

  @Singular(value = "alignedFollowerCANID")
  @NonNull
  public final Set<Integer> alignedFollowerCANIDs;

  @Singular(value = "opposedFollowerCANID")
  @NonNull
  public final Set<Integer> opposedFollowerCANIDs;
}
