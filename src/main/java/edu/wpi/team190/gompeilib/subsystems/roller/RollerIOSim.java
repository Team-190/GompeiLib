package edu.wpi.team190.gompeilib.subsystems.roller;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;

public class RollerIOSim implements RollerIO {
  private final DCMotorSim sim;

  private double appliedVolts;

  public RollerIOSim(RollerConstants consts) {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                consts.ROLLER_GEARBOX,
                0.004,
                consts.ROLLER_MOTOR_GEAR_RATIO),
            consts.ROLLER_GEARBOX);

    appliedVolts = 0.0;
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    appliedVolts = MathUtil.clamp(appliedVolts, -12.0, 12.0);

    sim.setInputVoltage(appliedVolts);
    sim.update(GompeiLib.getLoopPeriod());

    inputs.position = Rotation2d.fromRadians(sim.getAngularPositionRad());
    inputs.velocity = sim.getAngularVelocity();
    inputs.appliedVolts = appliedVolts;
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  public void setVoltage(double volts) {
    appliedVolts = volts;
  }
}