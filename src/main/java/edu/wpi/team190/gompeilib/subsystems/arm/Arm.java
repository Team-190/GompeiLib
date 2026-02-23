package edu.wpi.team190.gompeilib.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class Arm {
  public ArmIO io;
  public ArmIOInputsAutoLogged inputs;

  private final String aKitTopic;

  private final SysIdRoutine characterizationRoutine;

  @Getter private ArmState currentState;

  @Getter private double voltageGoalVolts;
  @Getter private Rotation2d positionGoal;

  public Arm(ArmIO io, Subsystem subsystem, int index) {
    this.io = io;
    this.inputs = new ArmIOInputsAutoLogged();

    aKitTopic = subsystem.getName() + "/Arms" + index;
    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.2).per(Second),
                Volts.of(3.5),
                Seconds.of(5),
                (state) -> Logger.recordOutput(aKitTopic + "/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)), null, subsystem));

    currentState = ArmState.IDLE;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    Logger.recordOutput(aKitTopic + "/State", currentState.name());
    Logger.recordOutput(aKitTopic + "/At Goal", atGoal());

    switch (currentState) {
      case OPEN_LOOP_VOLTAGE_CONTROL -> io.setVoltage(voltageGoalVolts);
      case CLOSED_LOOP_POSITION_CONTROL -> io.setPositionGoal(positionGoal);
    }
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
    return Commands.runOnce(
        () -> {
          currentState = ArmState.CLOSED_LOOP_POSITION_CONTROL;
          this.positionGoal = positionGoal;
        });
  }

  public Command setVoltage(double volts) {
    return Commands.runOnce(
        () -> {
          currentState = ArmState.OPEN_LOOP_VOLTAGE_CONTROL;
          this.voltageGoalVolts = volts;
        });
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

  public Command sysIdRoutine() {
    return Commands.sequence(
        Commands.runOnce(() -> currentState = ArmState.IDLE),
        characterizationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(1.0),
        characterizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(1.0),
        characterizationRoutine.dynamic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(1.0),
        characterizationRoutine.dynamic(SysIdRoutine.Direction.kReverse));
  }
}
