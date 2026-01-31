package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;

public class GenericFlywheelConstants {

  public final int[] CAN_IDS;

  public final boolean ON_CANIVORE;
  public final boolean ENABLE_FOC;

  public final double CURRENT_LIMIT;
  public final double MOMENT_OF_INERTIA;
  public final double GEAR_RATIO;

  public final DCMotor MOTOR_CONFIG;

  public final Gains GAINS;
  public final Constraints CONSTRAINTS;

  public final InvertedValue INVERSION;

  public final int[] COUNTERCLOCKWISE_CAN_IDS;
  public final int[] CLOCKWISE_CAN_IDS;

  public GenericFlywheelConstants(
      int[] CAN_IDS,
      boolean ON_CANIVORE,
      boolean ENABLE_FOC,
      double CURRENT_LIMIT,
      double MOMENT_OF_INERTIA,
      Gains GAINS,
      DCMotor MOTOR_CONFIG,
      Constraints CONSTRAINTS,
      double GEAR_RATIO,
      InvertedValue INVERSION,
      int[] COUNTERCLOCKWISE_CAN_IDS,
      int[] CLOCKWISE_CAN_IDS) {

    this.CAN_IDS = CAN_IDS;
    this.ON_CANIVORE = ON_CANIVORE;
    this.ENABLE_FOC = ENABLE_FOC;
    this.CURRENT_LIMIT = CURRENT_LIMIT;
    this.MOMENT_OF_INERTIA = MOMENT_OF_INERTIA;
    this.GAINS = GAINS;
    this.MOTOR_CONFIG = MOTOR_CONFIG;
    this.CONSTRAINTS = CONSTRAINTS;
    this.INVERSION = INVERSION;
    this.COUNTERCLOCKWISE_CAN_IDS = COUNTERCLOCKWISE_CAN_IDS;
    this.CLOCKWISE_CAN_IDS = CLOCKWISE_CAN_IDS;
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
