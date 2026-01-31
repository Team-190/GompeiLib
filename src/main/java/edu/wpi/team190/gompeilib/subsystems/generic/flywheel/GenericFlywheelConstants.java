package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;

public class GenericFlywheelConstants {

  public final int[] CAN_IDS;

  public final double CURRENT_LIMIT;
  public final double MOMENT_OF_INERTIA;

  public final DCMotor[] MOTOR_CONFIGS;

  public final Gains GAINS;
  public final Constraints CONSTRAINTS;
  public final double GEAR_RATIO;

  public final InvertedValue INVERSION;

  public final int NUM_MOTORS = 2;

  public GenericFlywheelConstants(
      int[] CAN_IDS,
      double CURRENT_LIMIT,
      double MOMENT_OF_INERTIA,
      double GEAR_RATIO,
      DCMotor[] MOTOR_CONFIGS,
      Gains GAINS,
      Constraints CONSTRAINTS,
      InvertedValue INVERSION) {
    this.CAN_IDS = CAN_IDS;
    this.CURRENT_LIMIT = CURRENT_LIMIT;
    this.MOMENT_OF_INERTIA = MOMENT_OF_INERTIA;
    this.MOTOR_CONFIGS = MOTOR_CONFIGS;
    this.GAINS = GAINS;
    this.CONSTRAINTS = CONSTRAINTS;
    this.INVERSION = INVERSION;
    this.GEAR_RATIO = GEAR_RATIO;
  }

  public record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kG,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA) {}

  public record Constraints(
      LoggedTunableNumber maxAccelerationRadiansPerSecondSquared,
      LoggedTunableNumber cruisingVelocityRadiansPerSecond,
      LoggedTunableNumber goalToleranceRadiansPerSecond) {}
}
