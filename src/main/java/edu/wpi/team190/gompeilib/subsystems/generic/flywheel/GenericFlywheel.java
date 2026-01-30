package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

public class GenericFlywheel {
  private final GenericFlywheelIO io;
  private final GenericFlywheelIOInputsAutoLogged inputs;

  private final String aKitTopic;
  private final SysIdRoutine characterizationRoutine;

  private GenericFlywheelState currentState;

  private double velocityGoalRadiansPerSecond;
  private double voltageGoalVolts;

  public GenericFlywheel(GenericFlywheelIO io, Subsystem subsystem, String name) {
    this.io = io;
    inputs = new GenericFlywheelIOInputsAutoLogged();

    aKitTopic = subsystem.getName() + "/" + " Flywheel" + name;

    characterizationRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second),
                Volts.of(3.5),
                Seconds.of(10),
                (state) -> Logger.recordOutput(aKitTopic + "/SysID State", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> io.setVoltage(voltage.in(Volts)), null, subsystem));

    velocityGoalRadiansPerSecond = 0;
    voltageGoalVolts = 0;

    currentState = GenericFlywheelState.IDLE;
    ;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    Logger.recordOutput(aKitTopic + "/Velocity Goal", velocityGoalRadiansPerSecond);
    Logger.recordOutput(aKitTopic + "/Voltage Goal", voltageGoalVolts);
    Logger.recordOutput(aKitTopic + "/Current State", currentState.name());
    Logger.recordOutput(aKitTopic + "/At Goal", io.atGoal());

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

  public Command setGoal(double velocityGoalRadiansPerSecond, boolean torqueControl) {
    return Commands.runOnce(
        () -> {
          currentState =
              torqueControl
                  ? GenericFlywheelState.VELOCITY_TORQUE_CONTROL
                  : GenericFlywheelState.VELOCITY_VOLTAGE_CONTROL;
          this.velocityGoalRadiansPerSecond = velocityGoalRadiansPerSecond;
        });
  }

  public Command setVoltage(double volts) {
    return Commands.runOnce(
        () -> {
          currentState = GenericFlywheelState.VOLTAGE_CONTROL;
          this.voltageGoalVolts = volts;
        });
  }

  public boolean atGoal() {
    return io.atGoal();
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
      double maxAccelerationRadiansPerSecondSquared,
      double cruisingVelocity,
      double goalToleranceRadiansPerSecond) {
    io.setProfile(
        maxAccelerationRadiansPerSecondSquared, cruisingVelocity, goalToleranceRadiansPerSecond);
  }

  public Command sysIdRoutine() {
    return Commands.sequence(
        Commands.runOnce(() -> currentState = GenericFlywheelState.IDLE),
        characterizationRoutine.dynamic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(1.0),
        characterizationRoutine.dynamic(SysIdRoutine.Direction.kReverse),
        Commands.waitSeconds(1.0),
        characterizationRoutine.quasistatic(SysIdRoutine.Direction.kForward),
        Commands.waitSeconds(1.0),
        characterizationRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
  }
}
