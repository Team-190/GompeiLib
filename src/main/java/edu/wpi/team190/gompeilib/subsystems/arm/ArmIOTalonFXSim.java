package edu.wpi.team190.gompeilib.subsystems.arm;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;

public class ArmIOTalonFXSim extends ArmIOTalonFX {
  private final SingleJointedArmSim armSim;

  private TalonFXSimState armController;

  public ArmIOTalonFXSim(ArmConstants constants) {
    super(constants);
    armSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                constants.ARM_PARAMETERS.MOTOR_CONFIG(),
                constants.ARM_PARAMETERS.MOMENT_OF_INERTIA(),
                constants.ARM_PARAMETERS.GEAR_RATIO()),
            constants.ARM_PARAMETERS.MOTOR_CONFIG(),
            constants.ARM_PARAMETERS.GEAR_RATIO(),
            constants.ARM_PARAMETERS.LENGTH_METERS(),
            constants.ARM_PARAMETERS.MIN_ANGLE().getRadians(),
            constants.ARM_PARAMETERS.MAX_ANGLE().getRadians(),
            true,
            constants.ARM_PARAMETERS.MIN_ANGLE().getRadians());
  }

  @Override
  @Trace
  public void updateInputs(ArmIOInputs inputs) {
    armController.setSupplyVoltage(RobotController.getBatteryVoltage());
    double armVoltage = armController.getMotorVoltage();

    armSim.setInputVoltage(armVoltage);

    armSim.update(GompeiLib.getLoopPeriod());

    Angle rotorPosition =
        Angle.ofBaseUnits(armSim.getAngleRads() * constants.ARM_PARAMETERS.GEAR_RATIO(), Radians);
    AngularVelocity rotorVelocity =
        AngularVelocity.ofBaseUnits(
            armSim.getVelocityRadPerSec() * constants.ARM_PARAMETERS.GEAR_RATIO(),
            RadiansPerSecond);
    armController.setRawRotorPosition(rotorPosition);
    armController.setRotorVelocity(rotorVelocity);

    super.updateInputs(inputs);
  }
}
