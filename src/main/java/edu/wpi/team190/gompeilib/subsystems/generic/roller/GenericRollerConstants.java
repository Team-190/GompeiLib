package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import lombok.Builder;

@Builder
public class GenericRollerConstants {
  public final int ROLLER_CAN_ID;
  public final double SUPPLY_CURRENT_LIMIT;
  public final DCMotor ROLLER_GEARBOX;
  public final double ROLLER_MOTOR_GEAR_RATIO;
  public final MomentOfInertia MOMENT_OF_INERTIA;
  @Builder.Default public final CANBus CAN_BUS = new CANBus();
}
