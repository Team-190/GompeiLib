package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;

public class GenericFlywheelIOTalonFXSim extends GenericFlywheelIOTalonFX {
  private final FlywheelSim flywheelSim;

  private final TalonFXSimState flywheelController;

  public GenericFlywheelIOTalonFXSim(GenericFlywheelConstants constants) {
    super(constants);
    flywheelSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                constants.MOTOR_CONFIG, constants.MOMENT_OF_INERTIA, constants.GEAR_RATIO),
            constants.MOTOR_CONFIG);

    flywheelController = super.talonFX.getSimState();
  }

  @Override
  @Trace
  public void updateInputs(GenericFlywheelIOInputs inputs) {
    flywheelController.setSupplyVoltage(RobotController.getBatteryVoltage());
    double flywheelVoltage = flywheelController.getMotorVoltage();

    flywheelSim.setInputVoltage(flywheelVoltage);

    flywheelSim.update(GompeiLib.getLoopPeriod());

    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(
            flywheelSim.getAngularVelocityRadPerSec() * constants.GEAR_RATIO, RadiansPerSecond);
    AngularAcceleration rotorAcceleration =
        AngularAcceleration.ofBaseUnits(
            flywheelSim.getAngularAccelerationRadPerSecSq() * constants.GEAR_RATIO,
            RadiansPerSecondPerSecond);
    flywheelController.setRotorVelocity(rotorVelocity);
    flywheelController.setRotorAcceleration(rotorAcceleration);

    super.updateInputs(inputs);
  }
}
