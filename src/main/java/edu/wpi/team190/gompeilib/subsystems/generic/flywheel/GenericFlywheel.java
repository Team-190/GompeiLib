package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import edu.wpi.team190.gompeilib.core.utility.sysid.CustomSysIdRoutine;
import edu.wpi.team190.gompeilib.core.utility.sysid.CustomUnits;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class GenericFlywheel {
  private final GenericFlywheelIO io;
  private final GenericFlywheelIOInputsAutoLogged inputs;

  private final String aKitTopic;
  private final CustomSysIdRoutine<CurrentUnit> torqueCharacterizationRoutine;
  private final CustomSysIdRoutine<VoltageUnit> voltageCharacterizationRoutine;

  @Getter private GenericFlywheelState currentState;

  @Getter private double velocityGoalRadiansPerSecond;
  @Getter private double currentFeedforward;
  @Getter private double voltageGoalVolts;

  private final DoubleSupplier velocityGoalOffset;

  public GenericFlywheel(
      GenericFlywheelIO io, Subsystem subsystem, DoubleSupplier velocityGoalOffset, String name) {
    this.io = io;
    inputs = new GenericFlywheelIOInputsAutoLogged();

    aKitTopic = subsystem.getName() + "/" + "Flywheel" + name;

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
            new CustomSysIdRoutine.Mechanism<>((amps) -> io.setAmps(amps.in(Amp)), subsystem),
            Amp.mutable(0));

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
                (volts) -> io.setVoltage(volts.in(Volts)), subsystem),
            Volts.mutable(0));

    velocityGoalRadiansPerSecond = 0;
    currentFeedforward = 0;
    voltageGoalVolts = 0;

    this.velocityGoalOffset = velocityGoalOffset;

    currentState = GenericFlywheelState.IDLE;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    Logger.recordOutput(aKitTopic + "/Velocity Goal", velocityGoalRadiansPerSecond);
    Logger.recordOutput(aKitTopic + "/Voltage Goal", voltageGoalVolts);
    Logger.recordOutput(aKitTopic + "/Current State", currentState.name());
    Logger.recordOutput(aKitTopic + "/At Goal", io.atGoal());
    Logger.recordOutput(aKitTopic + "/Flywheel Velocity Offset", velocityGoalOffset.getAsDouble());
    Logger.recordOutput(aKitTopic + "/Flywheel Velocity Magnitude", Math.abs(velocityGoalRadiansPerSecond));

    switch (currentState) {
      case VELOCITY_VOLTAGE_CONTROL:
        io.setVelocityVoltage(
            Math.max(
                    0,
                    (velocityGoalRadiansPerSecond + velocityGoalOffset.getAsDouble())
                        * Math.signum(velocityGoalRadiansPerSecond))
                * Math.signum(velocityGoalRadiansPerSecond));
        break;
      case VELOCITY_TORQUE_CONTROL:
        io.setVelocityTorque(
            Math.max(
                    0,
                    (velocityGoalRadiansPerSecond + velocityGoalOffset.getAsDouble())
                        * Math.signum(velocityGoalRadiansPerSecond))
                * Math.signum(velocityGoalRadiansPerSecond),
            currentFeedforward);
        break;
      case VOLTAGE_CONTROL:
        io.setVoltage(voltageGoalVolts);
        break;
      case STOP:
        io.stop();
        break;
      case IDLE:
        break;
    }
  }

  public Command setVelocityGoal(double velocityGoalRadiansPerSecond) {
    return Commands.runOnce(
        () -> {
          currentState = GenericFlywheelState.VELOCITY_VOLTAGE_CONTROL;
          this.velocityGoalRadiansPerSecond = velocityGoalRadiansPerSecond;
        });
  }

  public Command setVelocityGoal(DoubleSupplier velocityGoalRadiansPerSecond) {
    return Commands.run(
        () -> {
          currentState = GenericFlywheelState.VELOCITY_VOLTAGE_CONTROL;
          this.velocityGoalRadiansPerSecond = velocityGoalRadiansPerSecond.getAsDouble();
        });
  }

  public Command setVelocityGoal(double velocityGoalRadiansPerSecond, double feedforward) {
    return Commands.runOnce(
        () -> {
          currentState = GenericFlywheelState.VELOCITY_TORQUE_CONTROL;
          this.currentFeedforward = feedforward;
          this.velocityGoalRadiansPerSecond = velocityGoalRadiansPerSecond;
        });
  }

  public Command setVelocityGoal(
      DoubleSupplier velocityGoalRadiansPerSecond, DoubleSupplier feedforward) {
    return Commands.run(
        () -> {
          currentState = GenericFlywheelState.VELOCITY_TORQUE_CONTROL;
          this.velocityGoalRadiansPerSecond = velocityGoalRadiansPerSecond.getAsDouble();
          this.currentFeedforward = feedforward.getAsDouble();
        });
  }

  public Command setVoltage(double volts) {
    return Commands.runOnce(
        () -> {
          currentState = GenericFlywheelState.VOLTAGE_CONTROL;
          this.voltageGoalVolts = volts;
        });
  }

  public Command stop() {
    return Commands.runOnce(
        () -> {
          currentState = GenericFlywheelState.STOP;
        });
  }

  public boolean atGoal() {
    return io.atGoal();
  }

  public Command waitUntilAtGoal() {
    return Commands.waitUntil(io::atGoal);
  }

  public void setPID(GainSlot slot, double kP, double kD) {
    io.setPID(slot, kP, 0.0, kD);
  }

  public void setFeedForward(GainSlot slot, double kS, double kV, double kA) {
    io.setFeedforward(slot, kS, kV, kA);
  }

  public void setProfile(
      double maxAccelerationRadiansPerSecondSquared,
      double cruisingVelocityRadiansPerSecond,
      double goalToleranceRadiansPerSecond) {
    io.setProfile(
        maxAccelerationRadiansPerSecondSquared,
        cruisingVelocityRadiansPerSecond,
        goalToleranceRadiansPerSecond);
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
