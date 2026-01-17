package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class GenericFlywheel {
  private final GenericFlywheelIO io;
  private final GenericFlywheelIOInputsAutoLogged inputs;

  private final int index;
  private final String aKitTopic;

  private double velocityGoalRadiansPerSecond;
  private double voltageGoalVolts;

  private boolean isClosedLoop;

  public GenericFlywheel(GenericFlywheelIO io, Subsystem subsystem, int index) {
    this.io = io;
    inputs = new GenericFlywheelIOInputsAutoLogged();

    this.index = index;
    aKitTopic = subsystem.getName() + "/Flywheels" + index;

    velocityGoalRadiansPerSecond = 0;
    voltageGoalVolts = 0;

    isClosedLoop = true;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);
    if (isClosedLoop) {
      io.setVelocity(velocityGoalRadiansPerSecond);
    } else {
      io.setVoltage(voltageGoalVolts);
    }
  }

  public void setGoal(double velocityGoalRadiansPerSecond) {
    isClosedLoop = true;
    this.velocityGoalRadiansPerSecond = velocityGoalRadiansPerSecond;
  }

  public void setVoltage(double volts) {
    isClosedLoop = false;
    this.voltageGoalVolts = volts;
  }

  public boolean atGoal() {
    return io.atGoal();
  }

  public void setPID(double kP, double kD) {
    io.setPID(kP, 0.0, kD);
  }

  public void setFeedForward(double kS, double kV, double kA) {
    io.setFeedforward(kS, kV, kA);
  }

  public void setProfile(
      double maxAccelerationRadiansPerSecondSquared, double goalToleranceRadiansPerSecond) {
    io.setProfile(maxAccelerationRadiansPerSecondSquared, goalToleranceRadiansPerSecond);
  }

  public SysIdRoutine getCharacterization(
      double rampVoltage, double stepVoltage, double timeoutSeconds, Subsystem subsystem) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(rampVoltage).per(Second),
            Volts.of(stepVoltage),
            Seconds.of(timeoutSeconds),
            null),
        new SysIdRoutine.Mechanism((voltage) -> io.setVoltage(voltage.in(Volts)), null, subsystem));
  }
}
