package edu.wpi.team190.gompeilib.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.team190.gompeilib.core.utility.GainSlot;
import edu.wpi.team190.gompeilib.subsystems.arm.ArmIO.ArmIOInputs;

public class Arm {
  public ArmIO io;
  public ArmConstants armConstants;
  public ArmIOInputsAutoLogged inputs;
  private boolean isClosedLoop;

  private Rotation2d rotationGoal;

  public Arm(ArmConstants constants, ArmIO io) {
    this.io = io;
    this.armConstants = constants;
    this.inputs = new ArmIOInputsAutoLogged();
  }

  public void periodic() {
    io.updateInputs(inputs);

    if (isClosedLoop) io.setPositionGoal(rotationGoal);
  }

  public Rotation2d getArmPosition() {
    return inputs.position;
  }

  public void updateGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    io.updateGains(kP, kD, kS, kV, kA, kG);
  }

  public void updateConstraints(double maxAcceleration, double cruisingVelocity) {
    io.updateConstraints(maxAcceleration, cruisingVelocity);
  }

  public void updateInputs(ArmIOInputs inputs) {
    io.updateInputs(inputs);
  }

  public void setPosition(Rotation2d positionGoal) {
    inputs.position = positionGoal;
    isClosedLoop = true;
  }

  public void setPositionGoal(Rotation2d rotationGoal) {
      inputs.positionGoal = rotationGoal;
  }

  public void setVoltage(double volts) {
      io.setVoltage(volts);
  }

  public void setSlot(GainSlot slot){
      inputs.slot = slot;
  }

  public SysIdRoutine getCharacterization(
      double startingVoltage, double voltageIncrement, double timeSeconds, Subsystem subsystem) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(startingVoltage).per(Second),
            Volts.of(voltageIncrement),
            Seconds.of(timeSeconds),
            null),
        new SysIdRoutine.Mechanism(
            (voltage) -> io.setVoltage(voltage.in(Volts)), null, subsystem));
  }
}
