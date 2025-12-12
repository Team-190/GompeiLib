package edu.wpi.team190.gompeilib.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.subsystems.elevator.ElevatorIO.ElevatorIOInputs;
import java.util.Arrays;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
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
    elevatorConstants.lock.lock();
    io.updateInputs(inputs);
    elevatorConstants.lock.unlock();

    Logger.processInputs("Elevator", inputs);
    Logger.recordOutput("Elevator/Position", position);

    if (isClosedLoop) {
      io.setPositionGoal(positionGoalMeters);
    }
  }

  public void updateGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    io.updateGains(kP, kD, kS, kV, kA, kG);
  }

  public void updateConstraints(double maxAcceleration, double cruisingVelocity) {
    io.updateConstraints(maxAcceleration, cruisingVelocity);
  }

  public void updateInputs(ElevatorIOInputs inputs) {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", this.inputs);
  }

  public void setPosition(double positionMeters) {
    inputs.positionMeters = positionMeters;
  }

  public void setPositionGoal(double positionMeters) {
    inputs.positionMeters = positionMeters;
    isClosedLoop = true;
  }

  public void setVoltage(double volts) {
    Arrays.fill(inputs.appliedVolts, volts);
    isClosedLoop = false;
  }

  public double getPositionMeters() {
    return inputs.positionMeters;
  }
}
