package edu.wpi.team190.gompeilib.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.team190.gompeilib.core.utility.GainSlot;
import org.littletonrobotics.junction.Logger;

public class Arm {
  public ArmIO io;
  public ArmIOInputsAutoLogged inputs;

  private final String aKitTopic;

  public Arm(ArmIO io, Subsystem subsystem, int index) {
    this.io = io;
    this.inputs = new ArmIOInputsAutoLogged();

    aKitTopic = subsystem.getName() + "/Arms" + index;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);
  }

  public Rotation2d getArmPosition() {
    return inputs.position;
  }

  public void updateGains(
      double kP, double kD, double kS, double kV, double kA, double kG, GainSlot slot) {
    io.updateGains(kP, kD, kS, kV, kA, kG, slot);
  }

  public void updateConstraints(
      double maxAcceleration, double cruisingVelocity, double goalTolerance) {
    io.updateConstraints(maxAcceleration, cruisingVelocity, goalTolerance);
  }

  public Command setPosition(Rotation2d position) {
    return Commands.runOnce(() -> io.setPosition(position));
  }

  public Command setPositionGoal(Rotation2d positionGoal) {
    return Commands.runOnce(() -> io.setPositionGoal(positionGoal));
  }

  public Command setVoltage(double volts) {
    return Commands.runOnce(() -> io.setVoltage(volts));
  }

  public void setSlot(GainSlot slot) {
    io.setSlot(slot);
  }

  public boolean atGoal() {
    return io.atGoal();
  }

  public Command waitUntilAtGoal() {
    return Commands.waitUntil(this::atGoal);
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
