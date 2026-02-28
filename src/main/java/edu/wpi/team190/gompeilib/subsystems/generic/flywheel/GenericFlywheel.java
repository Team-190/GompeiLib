package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.team190.gompeilib.core.utility.Offset;
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

  @Getter private Offset<AngularVelocityUnit> velocityGoalRadiansPerSecond;
  @Getter private Offset<VoltageUnit> voltageGoalVolts;

  public GenericFlywheel(
      GenericFlywheelIO io, GenericFlywheelConstants constants, Subsystem subsystem, String name) {
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

    velocityGoalRadiansPerSecond =
        new Offset<>(
            RadiansPerSecond.of(0),
            constants.velocityOffsetStep,
            RadiansPerSecond.of(-constants.gearRatio * constants.motorConfig.freeSpeedRadPerSec),
            RadiansPerSecond.of(constants.gearRatio * constants.motorConfig.freeSpeedRadPerSec));
    voltageGoalVolts =
        new Offset<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12));

    currentState = GenericFlywheelState.IDLE;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    Logger.recordOutput(
        aKitTopic + "/Velocity Goal", velocityGoalRadiansPerSecond.getNewSetpoint());
    Logger.recordOutput(aKitTopic + "/Voltage Goal", voltageGoalVolts.getNewSetpoint());
    Logger.recordOutput(aKitTopic + "/Current State", currentState.name());
    Logger.recordOutput(aKitTopic + "/At Goal", io.atGoal());

    switch (currentState) {
      case VELOCITY_VOLTAGE_CONTROL:
        io.setVelocity(velocityGoalRadiansPerSecond.getNewSetpoint().in(RadiansPerSecond));
        break;
      case VELOCITY_TORQUE_CONTROL:
        io.setVelocityTorque(velocityGoalRadiansPerSecond.getNewSetpoint().in(RadiansPerSecond));
        break;
      case VOLTAGE_CONTROL:
        io.setVoltage(voltageGoalVolts.getNewSetpoint().baseUnitMagnitude());
        break;
      case STOP:
        io.stop();
        break;
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
          this.velocityGoalRadiansPerSecond.setSetpoint(
              RadiansPerSecond.of(velocityGoalRadiansPerSecond));
        });
  }

  public Command setGoal(DoubleSupplier velocityGoalRadiansPerSecond, boolean torqueControl) {
    return Commands.run(
        () -> {
          currentState =
              torqueControl
                  ? GenericFlywheelState.VELOCITY_TORQUE_CONTROL
                  : GenericFlywheelState.VELOCITY_VOLTAGE_CONTROL;
          this.velocityGoalRadiansPerSecond.setSetpoint(
              RadiansPerSecond.of(velocityGoalRadiansPerSecond.getAsDouble()));
        });
  }

  public Command setVoltage(double volts) {
    return Commands.runOnce(
        () -> {
          currentState = GenericFlywheelState.VOLTAGE_CONTROL;
          this.voltageGoalVolts.setSetpoint(Volts.of(volts));
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

  public void setPID(double kP, double kD) {
    io.setPID(kP, 0.0, kD);
  }

  public void setFeedForward(double kS, double kV, double kA) {
    io.setFeedforward(kS, kV, kA);
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
