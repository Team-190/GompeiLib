package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

public class GenericRoller {
  private final GenericRollerIO io;
  private final GenericRollerIOInputsAutoLogged inputs;

  private final String aKitTopic;

  @Setter @Getter private Voltage voltageGoal;

  public GenericRoller(
      GenericRollerIO io, Subsystem subsystem, GenericRollerConstants constants, String name) {
    this.io = io;
    inputs = new GenericRollerIOInputsAutoLogged();
    aKitTopic = subsystem.getName() + "/Roller" + name;

    voltageGoal = Volts.of(0.0);
  }

  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    Logger.recordOutput(aKitTopic + "/Voltage Goal", voltageGoal);

    io.setVoltageGoal(voltageGoal);
  }

  public boolean atVoltageGoal(Voltage voltageReference) {
    return io.atVoltageGoal(voltageReference);
  }

  public boolean atVoltageGoal() {
    return atVoltageGoal(voltageGoal);
  }
}
