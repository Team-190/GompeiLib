package edu.wpi.team190.gompeilib.subsystems.roller;

import edu.wpi.first.math.system.plant.DCMotor;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public class RollerConstants {
    public final int ROLLER_CAN_ID;
    public final double SUPPLY_CURRENT_LIMIT;
    public final DCMotor ROLLER_GEARBOX;
    public final double ROLLER_MOTOR_GEAR_RATIO;
}