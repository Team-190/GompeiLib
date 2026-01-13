package edu.wpi.team190.gompeilib.subsystems.roller;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.MomentOfInertia;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class RollerConstants {
  public final int ROLLER_CAN_ID;
  public final double SUPPLY_CURRENT_LIMIT;
  public final DCMotor ROLLER_GEARBOX;
  public final double ROLLER_MOTOR_GEAR_RATIO;
  public final MomentOfInertia MOMENT_OF_INERTIA;
  public final boolean ON_CANIVORE;
}
