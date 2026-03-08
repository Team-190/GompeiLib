package edu.wpi.team190.gompeilib.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
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

  @Getter private ArmState currentState;

  @Getter private Voltage voltageGoal;
  @Getter private Rotation2d positionGoal;

  private final SysIdRoutine characterizationRoutine;

  public Arm(ArmIO io, Subsystem subsystem, int index) {
    this.io = io;
    this.inputs = new ArmIOInputsAutoLogged();

    aKitTopic = subsystem.getName() + "/Arms" + index;

    currentState = ArmState.IDLE;

    voltageGoal = Volts.of(0.0);
    positionGoal = Rotation2d.fromRadians(0.0);

    characterizationRoutine =
            new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.of(1).per(Second),
                            Volts.of(9),
                            Seconds.of(12),
                            (state) -> Logger.recordOutput(aKitTopic + "/SysIdState", state.toString())),
                    new SysIdRoutine.Mechanism(
                            io::setVoltageGoal, null, subsystem));

  }

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

  public Rotation2d getArmPosition() {
    return inputs.position;
  }

  public void setVoltageGoal(Voltage voltageGoal) {
              currentState = ArmState.OPEN_LOOP_VOLTAGE_CONTROL;
              this.voltageGoal = voltageGoal;
  }

  public void setPositionGoal(Rotation2d positionGoal) {
          currentState = ArmState.CLOSED_LOOP_POSITION_CONTROL;
          this.positionGoal = positionGoal;
  }

  public boolean atVoltageGoal(Voltage voltageReference) {
    return io.atVoltageGoal(voltageReference);
  }

  public boolean atPositionGoal(Rotation2d positionReference) {
    return io.atPositionGoal(positionReference);
  }

  public boolean atVoltageGoal() {
    return atVoltageGoal(voltageGoal);
  }

  public boolean atPositionGoal() {
    return atPositionGoal(positionGoal);
  }

  public void setPosition(Rotation2d position) {
    io.setPosition(position);
  }

  public void setGainSlot(GainSlot slot) {
    io.setGainSlot(slot);
  }

  public Command waitUntilAtGoal() {
    return Commands.waitUntil(this::atPositionGoal);
  }

  public void updateGains(
          double kP, double kD, double kS, double kV, double kA, double kG, GainSlot slot) {
    io.updateGains(kP, kD, kS, kV, kA, kG, slot);
  }

  public void updateConstraints(
          AngularAcceleration maxAcceleration,
          AngularVelocity maxVelocity,
          Rotation2d goalTolerance) {
    io.updateConstraints(maxAcceleration, maxVelocity, goalTolerance);
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
