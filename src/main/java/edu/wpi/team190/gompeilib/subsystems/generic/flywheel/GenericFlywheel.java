package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.team190.gompeilib.core.utility.control.AngularVelocityConstraints;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.core.utility.sysid.CustomSysIdRoutine;
import edu.wpi.team190.gompeilib.core.utility.sysid.CustomUnits;
import java.util.function.Supplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class GenericFlywheel {
  private final GenericFlywheelIO io;
  private final GenericFlywheelIOInputsAutoLogged inputs;

  private final String aKitTopic;

  @Getter private GenericFlywheelState currentState;

  @Getter private AngularVelocity velocityGoal;
  @Getter private Voltage voltageGoal;
  @Getter private Current currentGoal;

  private final CustomSysIdRoutine<VoltageUnit> voltageCharacterizationRoutine;
  private final CustomSysIdRoutine<CurrentUnit> torqueCharacterizationRoutine;

  public GenericFlywheel(
      GenericFlywheelIO io, Subsystem subsystem, GenericFlywheelConstants constants, String name) {
    this.io = io;
    inputs = new GenericFlywheelIOInputsAutoLogged();

    aKitTopic = subsystem.getName() + "/" + "Flywheel" + name;

    currentState = GenericFlywheelState.IDLE;

    velocityGoal = RadiansPerSecond.of(0.0);
    voltageGoal = Volts.of(0.0);
    currentGoal = Amps.of(0.0);

    voltageCharacterizationRoutine =
        new CustomSysIdRoutine<>(
            new CustomSysIdRoutine.Config<VoltageUnit>(
                CustomUnits.voltsPerSecond.ofNative(0.5),
                Volts.of(8),
                Seconds.of(24),
                (state) ->
                    Logger.recordOutput(aKitTopic + "/Voltage SysID State", state.toString()),
                Volts),
            new CustomSysIdRoutine.Mechanism<>(
                (volts) -> io.setVoltageGoal(Volts.of(volts.in(Volts))), subsystem),
            Volts.mutable(0));

    torqueCharacterizationRoutine =
        new CustomSysIdRoutine<>(
            new CustomSysIdRoutine.Config<CurrentUnit>(
                CustomUnits.ampsPerSecond.ofNative(0.5),
                Amps.of(3.5),
                Seconds.of(10),
                (state) ->
                    Logger.recordOutput(
                        aKitTopic + "/Torque Current SysID State", state.toString()),
                Amp),
            new CustomSysIdRoutine.Mechanism<>(
                (amps) -> io.setCurrentGoal(Amps.of(amps.in(Amps))), subsystem),
            Amp.mutable(0));
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    Logger.recordOutput(aKitTopic + "/State", currentState.name());
    Logger.recordOutput(aKitTopic + "/Velocity Goal", velocityGoal);
    Logger.recordOutput(aKitTopic + "/Voltage Goal", voltageGoal);
    Logger.recordOutput(aKitTopic + "/Current Goal", currentGoal);
    Logger.recordOutput(aKitTopic + "/At Velocity Goal", atVelocityGoal());
    Logger.recordOutput(aKitTopic + "/At Voltage Goal", atVoltageGoal());
    Logger.recordOutput(aKitTopic + "/At Current Goal", atCurrentGoal());

    switch (currentState) {
      case VELOCITY_VOLTAGE_CONTROL:
        io.setVelocityGoal(velocityGoal);
        break;
      case VELOCITY_TORQUE_CONTROL:
        io.setVelocityGoal(velocityGoal, currentGoal);
        break;
      case VOLTAGE_CONTROL:
        io.setVoltageGoal(voltageGoal);
        break;
      case STOP:
        io.setVoltageGoal(Volts.of(0.0));
        break;
      case IDLE:
        break;
    }
  }

  public AngularVelocity getFlywheelVelocity() {
    return inputs.velocity;
  }

  public void setVoltageGoal(Voltage voltageGoal) {
    currentState = GenericFlywheelState.VOLTAGE_CONTROL;
    this.voltageGoal = voltageGoal;
  }

  public void setVelocityGoal(AngularVelocity velocityGoal) {
    currentState = GenericFlywheelState.VELOCITY_VOLTAGE_CONTROL;
    this.velocityGoal = velocityGoal;
  }

  public void setVelocityGoal(Supplier<AngularVelocity> velocityGoal) {
    currentState = GenericFlywheelState.VELOCITY_VOLTAGE_CONTROL;
    this.velocityGoal = velocityGoal.get();
  }

  public void setVelocityGoal(AngularVelocity velocityGoal, Current currentGoal) {
    currentState = GenericFlywheelState.VELOCITY_TORQUE_CONTROL;
    this.velocityGoal = velocityGoal;
    this.currentGoal = currentGoal;
  }

  public void setVelocityGoal(
      Supplier<AngularVelocity> velocityGoal, Supplier<Current> currentGoal) {
    currentState = GenericFlywheelState.VELOCITY_TORQUE_CONTROL;
    this.velocityGoal = velocityGoal.get();
    this.currentGoal = currentGoal.get();
  }

  public boolean atVoltageGoal(Voltage voltageReference) {
    return io.atVoltageGoal(voltageReference);
  }

  public boolean atVelocityGoal(AngularVelocity velocityReference) {
    return io.atVelocityGoal(velocityReference);
  }

  public boolean atCurrentGoal(Current currentReference) {
    return io.atCurrentGoal(currentReference);
  }

  public boolean atVoltageGoal() {
    return atVoltageGoal(voltageGoal);
  }

  public boolean atVelocityGoal() {
    return atVelocityGoal(velocityGoal);
  }

  public boolean atCurrentGoal() {
    return atCurrentGoal(currentGoal);
  }

  public Command waitUntilAtGoal() {
    return Commands.waitUntil(this::atVelocityGoal);
  }

  public void updateGains(Gains gains, GainSlot gainSlot) {
    io.updateGains(gains, gainSlot);
  }

  public void updateConstraints(AngularVelocityConstraints constraints) {
    io.updateConstraints(constraints);
  }

  public Command sysIdRoutineVoltage() {
    return Commands.sequence(
        Commands.runOnce(() -> currentState = GenericFlywheelState.IDLE),
        voltageCharacterizationRoutine.dynamic(CustomSysIdRoutine.Direction.kForward),
        Commands.waitSeconds(1.0),
        voltageCharacterizationRoutine.dynamic(CustomSysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(1.0),
        voltageCharacterizationRoutine.quasistatic(CustomSysIdRoutine.Direction.kForward),
        Commands.waitSeconds(1.0),
        voltageCharacterizationRoutine.quasistatic(CustomSysIdRoutine.Direction.kReverse));
  }

  public Command sysIdRoutineTorque() {
    return Commands.sequence(
        Commands.runOnce(() -> currentState = GenericFlywheelState.IDLE),
        torqueCharacterizationRoutine.dynamic(CustomSysIdRoutine.Direction.kForward),
        Commands.waitSeconds(1.0),
        torqueCharacterizationRoutine.dynamic(CustomSysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(1.0),
        torqueCharacterizationRoutine.quasistatic(CustomSysIdRoutine.Direction.kForward),
        Commands.waitSeconds(1.0),
        torqueCharacterizationRoutine.quasistatic(CustomSysIdRoutine.Direction.kReverse));
  }
}
