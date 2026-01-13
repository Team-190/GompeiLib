package edu.wpi.team190.gompeilib.subsystems.roller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Roller {
  private final RollerIO io;
  private final RollerIOInputsAutoLogged inputs;
  private final String aKitTopic;

  public Roller(RollerIO io, Subsystem subsystem, int index) {
    this.io = io;
    inputs = new RollerIOInputsAutoLogged();
    aKitTopic = subsystem.getName() + "/Rollers" + index;
  }

  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);
  }

  public Command runRoller(DoubleSupplier forward, DoubleSupplier reverse) {
    return Commands.runEnd(
        () -> io.setVoltage(12 * (forward.getAsDouble() - reverse.getAsDouble())),
        () -> io.setVoltage(0));
  }
}
