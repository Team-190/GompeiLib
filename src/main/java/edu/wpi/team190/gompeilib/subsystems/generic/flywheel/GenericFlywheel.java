package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.team190.gompeilib.subsystems.generic.flywheel.GenericFlywheelState.FlywheelState;

import org.littletonrobotics.junction.Logger;

public class GenericFlywheel {
  private final GenericFlywheelIO io;
  private final GenericFlywheelIOInputsAutoLogged inputs;

  private final String aKitTopic;

  private FlywheelState currentState;

  private double velocityGoalRadiansPerSecond;
  private double voltageGoalVolts;

  private boolean isClosedLoop;

  public GenericFlywheel(GenericFlywheelIO io, Subsystem subsystem, String name) {
    this.io = io;
    inputs = new GenericFlywheelIOInputsAutoLogged();

    aKitTopic = subsystem.getName() + "/" + name + " Flywheel";

    velocityGoalRadiansPerSecond = 0;
    voltageGoalVolts = 0;

    isClosedLoop = true;
    currentState = FlywheelState.IDLE;                                                                                                               ;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);
      switch (currentState) {
        case VELOCITY_VOLTAGE_CONTROL:
          io.setVelocity(velocityGoalRadiansPerSecond);
        case VELOCITY_TORQUE_CONTROL:
          io.setVelocityTorque(velocityGoalRadiansPerSecond);
        case VOLTAGE_CONTROL:
          io.setVoltage(voltageGoalVolts);
        case IDLE:
          break;
    }
  }

  public Command setGoal(double velocityGoalRadiansPerSecond) {
    return Commands.runOnce(
        () -> {
          isClosedLoop = true;
          this.velocityGoalRadiansPerSecond = velocityGoalRadiansPerSecond;
        });
  }

  public Command setVoltage(double volts) {
    return Commands.runOnce(
        () -> {
          isClosedLoop = false;
          this.voltageGoalVolts = volts;
        });
  }

  public Command waitUntilAtGoal() {
    return Commands.waitUntil(io::atGoal);
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

  public void updateConstraints(double maxAcceleration, double cruisingVelocity) {
    io.updateConstraints(maxAcceleration, cruisingVelocity);
  }

  public SysIdRoutine getCharacterization(
      double rampVoltage, double stepVoltage, double timeoutSeconds, Subsystem subsystem) {
    return new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.of(rampVoltage).per(Second),
            Volts.of(stepVoltage),
            Seconds.of(timeoutSeconds),
            (state) -> Logger.recordOutput(aKitTopic + "/SysID State", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> io.setVoltage(voltage.in(Volts)), null, subsystem));
  }
}
