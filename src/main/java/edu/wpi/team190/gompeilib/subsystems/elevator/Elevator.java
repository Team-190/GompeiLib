package edu.wpi.team190.gompeilib.subsystems.elevator;

import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.GainSlot;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  public final ElevatorIO io;
  public final ElevatorConstants elevatorConstants;
  public final ElevatorIOInputsAutoLogged inputs;

  private boolean isClosedLoop;
  private double position;
  private double positionGoalMeters;

  public Elevator(ElevatorConstants elevatorConstants, ElevatorIO io) {
    this.io = io;
    this.elevatorConstants = elevatorConstants;
    this.inputs = new ElevatorIOInputsAutoLogged();
  }

  @Trace
  public void periodic() {
    io.updateInputs(inputs);

    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/Position", position);

    if (isClosedLoop) {
      io.setPositionGoal(positionGoalMeters);
    }
  }

  public void updateGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    io.updateGains(kP, kD, kS, kV, kA, kG);
  }

  public void updateGains(
      double kP, double kD, double kS, double kV, double kA, double kG, GainSlot slot) {
    io.updateGains(kP, kD, kS, kV, kA, kG, slot);
  }

  public void updateConstraints(double maxAcceleration, double cruisingVelocity) {
    io.updateConstraints(maxAcceleration, cruisingVelocity);
  }

  public void setPosition(double positionMeters) {
    io.setPosition(positionMeters);
  }

  public void setPositionGoal(double positionMeters) {
    io.setPositionGoal(positionGoalMeters);
    isClosedLoop = true;
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
    isClosedLoop = false;
  }

  public double getPositionMeters() {
    return inputs.positionMeters;
  }
}
