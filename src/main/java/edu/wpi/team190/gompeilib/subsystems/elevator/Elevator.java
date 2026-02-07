package edu.wpi.team190.gompeilib.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.GainSlot;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  public final ElevatorIO io;
  public final ElevatorConstants elevatorConstants;
  public final ElevatorIOInputsAutoLogged inputs;

  private ElevatorState currentState;
  private double positionGoal;
  private double voltageGoal;

  private final String aKitTopic;
  private final SysIdRoutine characterizationRoutine;

  public Elevator(
      ElevatorConstants elevatorConstants, Subsystem subsystem, int index, ElevatorIO io) {
    this.io = io;
    this.elevatorConstants = elevatorConstants;
    this.inputs = new ElevatorIOInputsAutoLogged();

    currentState = ElevatorState.IDLE;
    aKitTopic = subsystem.getName() + "/Elevators" + index;
    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(3),
                Seconds.of(3),
                (state) -> Logger.recordOutput(aKitTopic + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> io.setVoltage(volts.in(Volts)), null, subsystem));

    positionGoal = 0;
    voltageGoal = 0;
  }

  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    Logger.recordOutput(aKitTopic + "/State", currentState.name());
    Logger.recordOutput(aKitTopic + "/At Goal", atGoal());

    switch (currentState) {
      case OPEN_LOOP_VOLTAGE_CONTROL -> io.setVoltage(voltageGoal);
      case CLOSED_LOOP_POSITION_CONTROL -> io.setPosition(positionGoal);
    }
  }

  public void updateGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    io.updateGains(kP, kD, kS, kV, kA, kG);
  }

  public void updateGains(
      double kP, double kD, double kS, double kV, double kA, double kG, GainSlot slot) {
    io.updateGains(kP, kD, kS, kV, kA, kG, slot);
  }

  public void updateConstraints(
      double maxAcceleration, double cruisingVelocity, double goalTolerance) {
    io.updateConstraints(maxAcceleration, cruisingVelocity, goalTolerance);
  }

  public void setPosition(double positionMeters) {
    io.setPosition(positionMeters);
  }

  public Command setPositionGoal(double positionMeters) {
    return Commands.runOnce(
        () -> {
          currentState = ElevatorState.CLOSED_LOOP_POSITION_CONTROL;
          positionGoal = positionMeters;
        });
  }

  public Command setVoltage(double volts) {
    return Commands.runOnce(
        () -> {
          currentState = ElevatorState.OPEN_LOOP_VOLTAGE_CONTROL;
          voltageGoal = volts;
        });
  }

  public boolean atGoal() {
    return io.atGoal();
  }

  public Command waitUntilAtGoal() {
    return Commands.waitUntil(this::atGoal);
  }

  public double getPositionMeters() {
    return inputs.positionMeters;
  }

  public Command runSysIdRoutine() {
    return Commands.sequence(
        Commands.runOnce(() -> currentState = ElevatorState.IDLE),
        characterizationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(1.0),
        characterizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(1.0),
        characterizationRoutine.dynamic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(1.0),
        characterizationRoutine.dynamic(SysIdRoutine.Direction.kReverse));
  }
}
