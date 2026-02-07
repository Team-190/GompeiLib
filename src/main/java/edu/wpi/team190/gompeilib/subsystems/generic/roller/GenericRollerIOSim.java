package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;

public class GenericRollerIOSim implements GenericRollerIO {
  private final DCMotorSim sim;

  private double appliedVolts;

  public GenericRollerIOSim(GenericRollerConstants consts) {
    sim =
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

    sim.setInputVoltage(appliedVolts);
    sim.update(GompeiLib.getLoopPeriod());

    inputs.position = Rotation2d.fromRadians(sim.getAngularPositionRad());
    inputs.velocity = sim.getAngularVelocity();
    inputs.appliedVolts = Volts.of(appliedVolts);
    inputs.supplyCurrent = Amps.of(sim.getCurrentDrawAmps());
  }

  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}
