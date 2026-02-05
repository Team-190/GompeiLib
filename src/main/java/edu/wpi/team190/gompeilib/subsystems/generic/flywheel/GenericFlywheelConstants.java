package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;

public class GenericFlywheelConstants {

  public final int LEADER_CAN_ID;
  public final InvertedValue LEADER_INVERSION;

  public final CANBus CAN_LOOP;
  public final boolean ENABLE_FOC;

  public final double CURRENT_LIMIT;
  public final double MOMENT_OF_INERTIA;
  public final double GEAR_RATIO;

  public final DCMotor MOTOR_CONFIG;

  public final Gains GAINS;
  public final Constraints CONSTRAINTS;

  public final int[] ALIGNED_FOLLOWER_CAN_IDS;
  public final int[] OPPOSED_FOLLOWER_CAN_IDS;

  public GenericFlywheelConstants(
      int LEADER_CAN_ID,
      InvertedValue LEADER_INVERSION,
      CANBus CAN_LOOP,
      boolean ENABLE_FOC,
      double CURRENT_LIMIT,
      double MOMENT_OF_INERTIA,
      Gains GAINS,
      DCMotor MOTOR_CONFIG,
      int[] ALIGNED_FOLLOWER_CAN_IDS,
      int[] OPPOSED_FOLLOWER_CAN_IDS,
      Constraints CONSTRAINTS,
      double GEAR_RATIO) {
    this.LEADER_CAN_ID = LEADER_CAN_ID;
    this.LEADER_INVERSION = LEADER_INVERSION;
    this.CAN_LOOP = CAN_LOOP;
    this.ENABLE_FOC = ENABLE_FOC;
    this.CURRENT_LIMIT = CURRENT_LIMIT;
    this.MOMENT_OF_INERTIA = MOMENT_OF_INERTIA;
    this.GAINS = GAINS;
    this.MOTOR_CONFIG = MOTOR_CONFIG;
    this.CONSTRAINTS = CONSTRAINTS;
    this.ALIGNED_FOLLOWER_CAN_IDS = ALIGNED_FOLLOWER_CAN_IDS;
    this.OPPOSED_FOLLOWER_CAN_IDS = OPPOSED_FOLLOWER_CAN_IDS;
    this.GEAR_RATIO = GEAR_RATIO;
  }

  public record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA) {}

  public record Constraints(
      LoggedTunableNumber maxAccelerationRadiansPerSecondSquared,
      LoggedTunableNumber cruisingVelocityRadiansPerSecond,
      LoggedTunableNumber goalToleranceRadiansPerSecond) {}
}
