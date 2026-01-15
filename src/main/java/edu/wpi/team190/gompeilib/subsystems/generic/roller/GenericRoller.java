package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import org.littletonrobotics.junction.Logger;

public class GenericRoller {
  private final GenericRollerIO io;
  private final RollerIOInputsAutoLogged inputs;
  private final String aKitTopic;

  private double voltageGoalVolts;

  public GenericRoller(GenericRollerIO io, Subsystem subsystem, int index) {
    this.io = io;
    inputs = new RollerIOInputsAutoLogged();
    aKitTopic = subsystem.getName() + "/Rollers" + index;
  }

  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    io.setVoltage(voltageGoalVolts);
  }

  public void setVoltage(double volts) {
    voltageGoalVolts = volts;
  }
}
