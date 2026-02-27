package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.team190.gompeilib.core.logging.Trace;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class GenericRoller {
  private final GenericRollerIO io;
  private final GenericRollerIOInputsAutoLogged inputs;
  private final String aKitTopic;

  @Getter private double voltageGoalVolts;

  private final DoubleSupplier voltageGoalOffset;

  public GenericRoller(
      GenericRollerIO io, Subsystem subsystem, DoubleSupplier voltageGoalOffset, String name) {
    this.io = io;
    inputs = new GenericRollerIOInputsAutoLogged();
    aKitTopic = subsystem.getName() + "/Roller" + name;

    voltageGoalVolts = 0;
    this.voltageGoalOffset = voltageGoalOffset;
  }

  @Trace
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(aKitTopic, inputs);

    Logger.recordOutput(aKitTopic + "/Voltage Goal", voltageGoalVolts);
    Logger.recordOutput(aKitTopic + "/Voltage Offset", voltageGoalOffset.getAsDouble());
    Logger.recordOutput(aKitTopic + "/Voltage Magnitude", Math.abs(voltageGoalVolts));

    io.setVoltage(
        MathUtil.clamp(
            Math.max(
                    0,
                    (voltageGoalVolts + voltageGoalOffset.getAsDouble())
                        * Math.signum(voltageGoalVolts))
                * Math.signum(voltageGoalVolts),
            -12.0,
            12.0));
  }

  public Command setVoltage(double volts) {
    return Commands.runOnce(() -> voltageGoalVolts = volts);
  }
}
