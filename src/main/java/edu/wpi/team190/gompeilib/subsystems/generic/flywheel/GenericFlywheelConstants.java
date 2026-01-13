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

  public final DCMotor[] MOTOR_CONFIGS;

  public final Gains GAINS;
  public final Constraints CONSTRAINTS;
  public final GenericFlywheelParameters GENERIC_FLYWHEEL_PARAMETERS;
  public final InvertedValue INVERSION;

  public GenericFlywheelConstants(
      int[] CAN_IDS,
      boolean ON_CANIVORE,
      boolean ENABLE_FOC,
      double CURRENT_LIMIT,
      double MOMENT_OF_INERTIA,
      DCMotor[] MOTOR_CONFIGS,
      Gains GAINS,
      Constraints CONSTRAINTS,
      GenericFlywheelParameters GENERIC_FLYWHEEL_PARAMETERS,
      double GEAR_RATIO,
      InvertedValue INVERSION) {
    this.CAN_IDS = CAN_IDS;
    this.ON_CANIVORE = ON_CANIVORE;
    this.ENABLE_FOC = ENABLE_FOC;
    this.CURRENT_LIMIT = CURRENT_LIMIT;
    this.MOMENT_OF_INERTIA = MOMENT_OF_INERTIA;
    this.MOTOR_CONFIGS = MOTOR_CONFIGS;
    this.GAINS = GAINS;
    this.CONSTRAINTS = CONSTRAINTS;
    this.GENERIC_FLYWHEEL_PARAMETERS = GENERIC_FLYWHEEL_PARAMETERS;
    this.GEAR_RATIO = GEAR_RATIO;
    this.INVERSION = INVERSION;
  }

  public record GenericFlywheelParameters(
      DCMotor MOTOR_CONFIG,
      double CARRIAGE_MASS_KG,
      double MIN_HEIGHT_METERS,
      double MAX_HEIGHT_METERS,
      int NUM_MOTORS) {}

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
