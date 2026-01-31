package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import org.littletonrobotics.junction.Logger;

public class GenericRoller {
  private final GenericRollerIO io;
  private final RollerIOInputsAutoLogged inputs;
  private final String aKitTopic;

  private double voltageGoalVolts;

  public GenericRoller(GenericRollerIO io, Subsystem subsystem, String name) {
    this.io = io;
    inputs = new RollerIOInputsAutoLogged();
    aKitTopic = subsystem.getName() + "/" + name + " Rollers";
  }

  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    io.setVoltage(voltageGoalVolts);
  }

  public Command setVoltage(double volts) {
    return Commands.runOnce(
        () -> {
          voltageGoalVolts = volts;
        });
  }
}
