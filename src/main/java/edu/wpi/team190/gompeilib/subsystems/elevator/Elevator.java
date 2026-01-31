package edu.wpi.team190.gompeilib.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.GainSlot;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  public final ElevatorIO io;
  public final ElevatorConstants elevatorConstants;
  public final ElevatorIOInputsAutoLogged inputs;

  private final String aKitTopic;

  public Elevator(
      ElevatorConstants elevatorConstants, Subsystem subsystem, int index, ElevatorIO io) {
    this.io = io;
    this.elevatorConstants = elevatorConstants;
    this.inputs = new ElevatorIOInputsAutoLogged();

    aKitTopic = subsystem.getName() + "/Elevators" + index;
  }

  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);
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
    io.setPositionGoal(positionMeters);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public double getPositionMeters() {
    return inputs.positionMeters;
  }

  public SysIdRoutine getCharacterization(
      double rampVoltage, double stepVoltage, double timeoutSeconds, Subsystem subsystem) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(rampVoltage).per(Second),
            Volts.of(stepVoltage),
            Seconds.of(timeoutSeconds),
            (state) -> Logger.recordOutput(aKitTopic + "/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> io.setVoltage(voltage.in(Volts)), null, subsystem));
  }
}
