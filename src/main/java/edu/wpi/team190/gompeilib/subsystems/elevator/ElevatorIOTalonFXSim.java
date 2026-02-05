package edu.wpi.team190.gompeilib.subsystems.elevator;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.logging.Trace;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

public class ElevatorIOTalonFXSim extends ElevatorIOTalonFX {
    private final ElevatorSim elevatorSim;

    private TalonFXSimState elevatorController;

    public ElevatorIOTalonFXSim(ElevatorConstants constants) {
        super(constants);
        elevatorSim =
                new ElevatorSim(
                        LinearSystemId.createElevatorSystem(
                                constants.ELEVATOR_PARAMETERS.ELEVATOR_MOTOR_CONFIG(),
                                constants.ELEVATOR_PARAMETERS.CARRIAGE_MASS_KG(),
                                constants.DRUM_RADIUS,
                                constants.ELEVATOR_GEAR_RATIO),
                        constants.ELEVATOR_PARAMETERS.ELEVATOR_MOTOR_CONFIG(),
                        constants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS(),
                        constants.ELEVATOR_PARAMETERS.MAX_HEIGHT_METERS(),
                        true,
                        constants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS());
    }

    @Override
    @Trace
    public void updateInputs(ElevatorIOInputs inputs) {
        elevatorController.setSupplyVoltage(RobotController.getBatteryVoltage());
        double elevatorVoltage = elevatorController.getMotorVoltage();

        elevatorSim.setInputVoltage(elevatorVoltage);

        elevatorSim.update(GompeiLib.getLoopPeriod());

        Angle rotorPosition = Angle.ofBaseUnits(elevatorSim.getPositionMeters() * constants.ELEVATOR_GEAR_RATIO * constants.DRUM_RADIUS, Radians);
        AngularVelocity rotorVelocity = AngularVelocity.ofBaseUnits(elevatorSim.getVelocityMetersPerSecond() * constants.ELEVATOR_GEAR_RATIO * constants.DRUM_RADIUS, RadiansPerSecond);
        elevatorController.setRawRotorPosition(rotorPosition);
        elevatorController.setRotorVelocity(rotorVelocity);

        super.updateInputs(inputs);
    }
}
