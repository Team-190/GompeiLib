package edu.wpi.team190.gompeilib.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.team190.gompeilib.core.utility.LoggedTunableNumber;

public class ArmConstants {
  public final int ARM_CAN_ID;
  public boolean ON_CANIVORE;
  public final ArmParameters ARM_PARAMETERS;
  public final Gains SLOT0_GAINS;
  public final Gains SLOT1_GAINS;
  public final Gains SLOT2_GAINS;

  public final Constraints CONSTRAINTS;

  public final CurrentLimits CURRENT_LIMITS;

  public ArmConstants(
      int ARM_CAN_ID,
      ArmParameters ARM_PARAMETERS,
      Gains SLOT0_GAINS,
      Gains SLOT1_GAINS,
      Gains SLOT2_GAINS,
      Constraints CONSTRAINTS,
      CurrentLimits CURRENT_LIMITS) {
    this.ARM_CAN_ID = ARM_CAN_ID;
    this.ARM_PARAMETERS = ARM_PARAMETERS;
    this.SLOT0_GAINS = SLOT0_GAINS;
    this.SLOT1_GAINS = SLOT1_GAINS;
    this.SLOT2_GAINS = SLOT2_GAINS;
    this.CONSTRAINTS = CONSTRAINTS;
    this.CURRENT_LIMITS = CURRENT_LIMITS;
  }

  public static record ArmParameters(
      DCMotor MOTOR_CONFIG,
      Rotation2d MIN_ANGLE,
      Rotation2d MAX_ANGLE,
      int NUM_MOTORS,
      double GEAR_RATIO,
      double LENGTH_METERS,
      double MASS_KG,
      double MOMENT_OF_INERTIA_KG_M_SQUARED) {}

  public static record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kG,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA) {}

  public static record CurrentLimits(
      double ARM_SUPPLY_CURRENT_LIMIT,
      double ARM_STATOR_CURRENT_LIMIT,
      double ARM_TORQUE_CURRENT_LIMIT) {}

  public static record Constraints(
      LoggedTunableNumber MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED,
      LoggedTunableNumber CRUISING_VELOCITY_ROTATIONS_PER_SECOND,
      LoggedTunableNumber GOAL_TOLERANCE_RADIANS) {}
}
