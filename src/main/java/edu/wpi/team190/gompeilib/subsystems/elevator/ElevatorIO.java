package edu.wpi.team190.gompeilib.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {

    @AutoLog
    public static class ElevatorIOInputs {
        public double positionMeters = 0.0;
        public double velocityMetersPerSecond = 0.0;
        public double accelerationMetersPerSecondSquared = 0.0;

    public double[] appliedVolts = new double[] {};
    public double[] supplyCurrentAmps = new double[] {};
    public double[] torqueCurrentAmps = new double[] {};
    public double[] temperatureCelsius = new double[] {};

    public double positionGoalMeters = 0.0;
    public double positionSetpointMeters = 0.0;
    public double positionErrorMeters = 0.0;
  }

  /**
   * Updates the inputs for the elevator.
   *
   * @param inputs The inputs to update.
   */
  public void updateInputs(ElevatorIOInputs inputs);

  /**
   * Sets the position of the elevator.
   *
   * @param positionMeters The position to set in meters.
   */
  public void setPosition(double positionMeters);

  /**
   * Sets the position goal of the elevator.
   *
   * @param positionMeters The position goal of the elevator in meters.
   */
  public void setPositionGoal(double positionMeters);

  /**
   * Sets the voltage for the elevator.
   *
   * @param volts The voltage of the elevator in volts.
   */
  public void setVoltage(double volts);

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
  public void updateGains(double kP, double kD, double kS, double kV, double kA, double kG);

  /**
   * Sets the constraints for the elevator.
   *
   * @param maxAcceleration the max acceleration of the elevator.
   * @param cruisingVelocity the cruising velocity
   */
  public void updateConstraints(double maxAcceleration, double cruisingVelocity);
}
