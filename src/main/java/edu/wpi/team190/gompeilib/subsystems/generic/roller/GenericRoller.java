package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import edu.wpi.team190.gompeilib.core.utility.Offset;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class GenericRoller {
  private final GenericRollerIO io;
  private final GenericRollerIOInputsAutoLogged inputs;
  private final String aKitTopic;

  @Getter private Offset<VoltageUnit> voltageGoalVolts;

  public GenericRoller(
      GenericRollerIO io, Subsystem subsystem, GenericRollerConstants constants, String name) {
    this.io = io;
    inputs = new GenericRollerIOInputsAutoLogged();
    aKitTopic = subsystem.getName() + "/Roller" + name;

    voltageGoalVolts =
        new Offset<>(Volts.of(0), constants.voltageOffsetStep, Volts.of(-12), Volts.of(12));
  }

  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    Logger.recordOutput(aKitTopic + "/Voltage Goal", voltageGoalVolts.getNewSetpoint());

    Logger.recordOutput(aKitTopic + "/Voltage Offset", voltageGoalVolts.getOffset());
    Logger.recordOutput(
        aKitTopic + "/Voltage Magnitude",
        Math.abs(voltageGoalVolts.getSetpoint().baseUnitMagnitude()));

    io.setVoltage(voltageGoalVolts.getNewSetpoint().baseUnitMagnitude());
  }

  public Command setVoltage(Voltage volts) {
    return Commands.runOnce(() -> voltageGoalVolts.setSetpoint(volts));
  }

  public Command setVoltage(double volts) {
    return setVoltage(Volts.of(volts));
  }

  public Command incrementVoltageOffset() {
    return Commands.runOnce(voltageGoalVolts::increment);
  }

  public Command decrementVoltageOffset() {
    return Commands.runOnce(voltageGoalVolts::decrement);
  }

  public Command resetVoltageOffset() {
    return Commands.runOnce(voltageGoalVolts::reset);
  }
}
