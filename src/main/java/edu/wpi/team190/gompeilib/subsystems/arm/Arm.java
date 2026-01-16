package edu.wpi.team190.gompeilib.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.team190.gompeilib.core.utility.GainSlot;

public class Arm {
  public ArmIO io;
  public ArmIOInputsAutoLogged inputs;
  private boolean isClosedLoop;

  private Rotation2d rotationGoal;

  public Arm(ArmIO io) {
    this.io = io;
    this.inputs = new ArmIOInputsAutoLogged();
  }

  public void periodic() {
    io.updateInputs(inputs);

    if (isClosedLoop)
      io.setPositionGoal(rotationGoal);
  }

  public Rotation2d getArmPosition() {
    return inputs.position;
  }

  public void updateGains(
      double kP, double kD, double kS, double kV, double kA, double kG, GainSlot slot) {
    io.updateGains(kP, kD, kS, kV, kA, kG, slot);
  }

  public void updateConstraints(double maxAcceleration, double cruisingVelocity) {
    io.updateConstraints(maxAcceleration, cruisingVelocity);
  }

  public void setPosition(Rotation2d positionGoal) {
    io.setPositionGoal(positionGoal);
  }

  public void setPositionGoal(Rotation2d positionGoal) {
    io.setPositionGoal(positionGoal);
    isClosedLoop = true;
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setSlot(GainSlot slot) {
    io.setSlot(slot);
  }

  public SysIdRoutine getCharacterization(
      double startingVoltage, double voltageIncrement, double timeSeconds, Subsystem subsystem) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(startingVoltage).per(Second),
            Volts.of(voltageIncrement),
            Seconds.of(timeSeconds),
            null),
        new SysIdRoutine.Mechanism((voltage) -> io.setVoltage(voltage.in(Volts)), null, subsystem));
  }
}
