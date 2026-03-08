package edu.wpi.team190.gompeilib.subsystems.elevator;

import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.littletonrobotics.junction.AutoLog;

import static edu.wpi.first.units.Units.*;

public interface ElevatorIO {

  @AutoLog
  public static class ElevatorIOInputs {
    public Distance position = Meters.of(0.0);
    public LinearVelocity velocity = MetersPerSecond.of(0.0);
    public LinearAcceleration acceleration = MetersPerSecondPerSecond.of(0.0);

    public Voltage[] appliedVolts = new Voltage[] {};
    public Current[] supplyCurrentAmps = new Current[] {};
    public Current[] torqueCurrentAmps = new Current[] {};
    public Temperature[] temperatureCelsius = new Temperature[] {};

    public Distance positionGoalMeters = Meters.of(0.0);
    public Distance positionSetpointMeters = Meters.of(0.0);
    public Distance positionErrorMeters = Meters.of(0.0);
  }

  /**
   * Updates the inputs for the elevator.
   *
   * @param inputs The inputs to update.
   */
  default void updateInputs(ElevatorIOInputs inputs) {}

  /**
   * Sets the voltage for the elevator.
   *
   * @param voltageGoal The voltage of the elevator in volts.
   */
  default void setVoltageGoal(Voltage voltageGoal) {}

  /**
   * Sets the position goal of the elevator.
   *
   * @param positionGoal The position goal of the elevator in meters.
   */
  default void setPositionGoal(Distance positionGoal) {}

  /**
   * Checks if the voltage of the elevator matches the volts argument
   *
   * @param voltageReference The voltage to check against
   * @return True if the voltage matches, false otherwise
   */
  default boolean atVoltageGoal(Voltage voltageReference) {
    return false;
  }

  /**
   * Checks if the position of the subsystem matches the positionGoal argument
   *
   * @param positionReference the position to check against
   * @return True if the position matches, false otherwise
   */
  default boolean atPositionGoal(Distance positionReference) {
    return false;
  }

  /**
   * Sets the position of the elevator.
   *
   * @param position The position to set.
   */
  default void setPosition(Distance position) {}

  /**
   * @param gainSlot The CTRE gain slot to set the elevator to.
   */
  default void setGainSlot(GainSlot gainSlot) {}

  /**
   * Sets the gains for the elevator.
   *
   * @param kP the proportional gain.
   * @param kD the derivative gain.
   * @param kS the static gain.
   * @param kV the velocity gain.
   * @param kA the acceleration gain.
   * @param kG the gravity gain.
   */

  default void updateGains(
      double kP, double kD, double kS, double kV, double kA, double kG, GainSlot slot) {}

  /**
   * Sets the constraints for the elevator.
   *
   * @param maxAcceleration the max acceleration of the elevator.
   * @param maxVelocity the max velocity of the arm.
   * @param goalTolerance the acceptable tolerance for elevator control.
   */
  default void updateConstraints(
      LinearAcceleration maxAcceleration, LinearVelocity maxVelocity, Distance goalTolerance) {}
}
