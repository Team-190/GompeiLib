package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import lombok.Builder;
import lombok.NonNull;

@Builder(setterPrefix = "with")
public class GenericRollerConstants {
  @NonNull public final Integer rollerCANID;
  @NonNull public final Double supplyCurrentLimit;
  @NonNull public final DCMotor rollerGearbox;
  @NonNull public final Double rollerMotorGearRatio;
  @NonNull public final MomentOfInertia momentOfInertia;
  @NonNull public final NeutralModeValue neutralMode;
  @NonNull public final CANBus canBus;
}
