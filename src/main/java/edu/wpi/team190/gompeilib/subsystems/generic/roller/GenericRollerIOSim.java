package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import java.util.Arrays;

public class GenericRollerIOSim implements GenericRollerIO {
  private final DCMotorSim motorSim;

  private double appliedVolts;

  public GenericRollerIOSim(GenericRollerConstants consts) {
    motorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                consts.rollerGearbox,
                consts.momentOfInertia.baseUnitMagnitude(),
                consts.rollerMotorGearRatio),
            consts.rollerGearbox);

    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(GenericRollerIOInputs inputs) {
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    motorSim.setInputVoltage(appliedVolts);
    motorSim.update(GompeiLib.getLoopPeriod());

    inputs.positionRadians =
        Rotation2d.fromRadians(
            motorSim.getAngularVelocityRadPerSec() * 1 / GompeiLib.getLoopPeriod());
    inputs.velocityRadiansPerSecond = motorSim.getAngularVelocityRadPerSec();
    Arrays.fill(inputs.appliedVolts, appliedVolts);
    Arrays.fill(inputs.supplyCurrentAmps, motorSim.getCurrentDrawAmps());
    Arrays.fill(inputs.torqueCurrentAmps, motorSim.getCurrentDrawAmps());
  }

  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
