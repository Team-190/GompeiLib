package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class GenericRoller {
  private final GenericRollerIO io;
  private final GenericRollerIOInputsAutoLogged inputs;
  private final String aKitTopic;

  @Getter private double voltageGoalVolts;

  public GenericRoller(GenericRollerIO io, Subsystem subsystem, String name) {
    this.io = io;
    inputs = new GenericRollerIOInputsAutoLogged();
    aKitTopic = subsystem.getName() + "/Roller" + name;
  }

  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    Logger.recordOutput(aKitTopic + "/Voltage Goal", voltageGoalVolts);

    io.setVoltage(voltageGoalVolts);
  }

  public Command setVoltage(double volts) {
    return Commands.runOnce(() -> voltageGoalVolts = volts);
  }
}
