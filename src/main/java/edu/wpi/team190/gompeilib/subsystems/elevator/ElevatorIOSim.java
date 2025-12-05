package edu.wpi.team190.gompeilib.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import java.util.Arrays;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim sim;

  private final ProfiledPIDController feedback;
  private ElevatorFeedforward feedforward;

  private double appliedVolts;
  private boolean isClosedLoop;

  private final int numMotors;

  public ElevatorIOSim(ElevatorConstants constants) {
    sim =
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

    feedback =
        new ProfiledPIDController(
            constants.SLOT0_GAINS.kP().get(),
            0,
            constants.SLOT0_GAINS.kD().get(),
            new TrapezoidProfile.Constraints(
                constants.CONSTRAINTS.cruisingVelocityMetersPerSecond().get(),
                constants.CONSTRAINTS.maxAccelerationMetersPerSecondSquared().get()));

    feedforward =
        new ElevatorFeedforward(
            constants.SLOT0_GAINS.kS().get(),
            constants.SLOT0_GAINS.kG().get(),
            constants.SLOT0_GAINS.kV().get(),
            constants.SLOT0_GAINS.kA().get());

    appliedVolts = 0;
    isClosedLoop = true;

    numMotors = constants.ELEVATOR_PARAMETERS.NUM_MOTORS();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (isClosedLoop) {
      appliedVolts =
          feedback.calculate(sim.getPositionMeters())
              + feedforward.calculate((feedback.getSetpoint().velocity));
    }

    appliedVolts = MathUtil.clamp(appliedVolts, -12, 12);

    sim.setInputVoltage(appliedVolts);
    sim.update(0.02); // TODO: Replace with a constant

    inputs.positionSetpointMeters = sim.getPositionMeters();
    inputs.velocityMetersPerSecond = sim.getVelocityMetersPerSecond();
    inputs.accelerationMetersPerSecondSquared =
        -1; // TODO: Replace with calculation based on velocity

    Arrays.fill(inputs.appliedVolts, appliedVolts);
    Arrays.fill(inputs.supplyCurrentAmps, sim.getCurrentDrawAmps());
    Arrays.fill(inputs.torqueCurrentAmps, sim.getCurrentDrawAmps());
    Arrays.fill(inputs.temperatureCelsius, 0);

    inputs.positionGoalMeters = feedback.getGoal().position;
    inputs.positionSetpointMeters = feedback.getSetpoint().position;
    inputs.positionErrorMeters = feedback.getPositionError();
  }

  @Override
  public void setPosition(double positionMeters) {
    sim.setState(positionMeters, sim.getVelocityMetersPerSecond());
  }

  @Override
  public void setPositionGoal(double positionMeters) {
    feedback.setGoal(positionMeters);
    isClosedLoop = true;
  }

  @Override
  public void setVoltage(double volts) {
    isClosedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void updateGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    feedback.setPID(kP, 0, kD);
    feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
  }

  @Override
  public void updateConstraints(double maxAcceleration, double cruisingVelocity) {
    feedback.setConstraints(new TrapezoidProfile.Constraints(cruisingVelocity, maxAcceleration));
  }
}
