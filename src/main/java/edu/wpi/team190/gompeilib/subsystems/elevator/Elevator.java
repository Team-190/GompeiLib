package edu.wpi.team190.gompeilib.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.LinearConstraints;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import org.littletonrobotics.junction.Logger;

public class Elevator {
  public final ElevatorIO io;
  public final ElevatorIOInputsAutoLogged inputs;

  private final String aKitTopic;

  private ElevatorState currentState;

  private Voltage voltageGoal;
  private Distance positionGoal;

  private final SysIdRoutine characterizationRoutine;

  public final ElevatorConstants constants;

  public Elevator(ElevatorConstants constants, Subsystem subsystem, int index, ElevatorIO io) {
    this.io = io;
    this.inputs = new ElevatorIOInputsAutoLogged();

    aKitTopic = subsystem.getName() + "/Elevators" + index;

    currentState = ElevatorState.IDLE;

    voltageGoal = Volts.of(0.0);
    positionGoal = Meters.of(0.0);

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1).per(Second),
                Volts.of(3),
                Seconds.of(3),
                (state) -> Logger.recordOutput(aKitTopic + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(io::setVoltageGoal, null, subsystem));

    this.constants = constants;
  }

  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    Logger.recordOutput(aKitTopic + "/State", currentState.name());
    Logger.recordOutput(aKitTopic + "/Voltage Goal", voltageGoal);
    Logger.recordOutput(aKitTopic + "/Position Goal", positionGoal);
    Logger.recordOutput(aKitTopic + "/At Voltage Goal", atVoltageGoal());
    Logger.recordOutput(aKitTopic + "/At Position Goal", atPositionGoal());

    switch (currentState) {
      case OPEN_LOOP_VOLTAGE_CONTROL -> io.setVoltageGoal(voltageGoal);
      case CLOSED_LOOP_POSITION_CONTROL -> io.setPositionGoal(positionGoal);
    }
  }

  public Distance getElevatorPosition() {
    return inputs.position;
  }

  public void setVoltageGoal(Voltage voltageGoal) {
    currentState = ElevatorState.OPEN_LOOP_VOLTAGE_CONTROL;
    this.voltageGoal = voltageGoal;
  }

  public void setPositionGoal(Distance positionGoal) {
    currentState = ElevatorState.CLOSED_LOOP_POSITION_CONTROL;
    this.positionGoal = positionGoal;
  }

  public boolean atVoltageGoal(Voltage voltageReference) {
    return voltageGoal.isNear(voltageReference, Millivolts.of(500));
  }

  public boolean atPositionGoal(Distance positionReference) {
    return positionGoal.isNear(positionReference, constants.constraints.goalTolerance().get());
  }

  public boolean atVoltageGoal() {
    return atVoltageGoal(voltageGoal);
  }

  public boolean atPositionGoal() {
    return atPositionGoal(positionGoal);
  }

  public void setPosition(Distance position) {
    io.setPosition(position);
  }

  public void setGainSlot(GainSlot gainSlot) {
    io.setGainSlot(gainSlot);
  }

  public Command waitUntilAtGoal() {
    return Commands.waitUntil(this::atPositionGoal);
  }

  public void updateGains(Gains gains, GainSlot slot) {
    io.updateGains(gains, slot);
  }

  public void updateConstraints(LinearConstraints constraints) {
    io.updateConstraints(constraints);
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
