package edu.wpi.team190.gompeilib.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import java.util.Arrays;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim sim;

  private final ProfiledPIDController feedback;
  private ElevatorFeedforward feedforward;

  private double appliedVolts;
  private boolean isClosedLoop;

  private final ElevatorConstants constants;

  public ElevatorIOSim(ElevatorConstants constants) {
    sim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                constants.elevatorParameters.ELEVATOR_MOTOR_CONFIG(),
                constants.elevatorParameters.CARRIAGE_MASS_KG(),
                constants.drumRadius,
                constants.elevatorGearRatio),
            constants.elevatorParameters.ELEVATOR_MOTOR_CONFIG(),
            constants.elevatorParameters.MIN_HEIGHT_METERS(),
            constants.elevatorParameters.MAX_HEIGHT_METERS(),
            true,
            constants.elevatorParameters.MIN_HEIGHT_METERS());

    feedback =
        new ProfiledPIDController(
            constants.slot0Gains.kP().get(),
            0,
            constants.slot0Gains.kD().get(),
            new TrapezoidProfile.Constraints(
                constants.constraints.maxVelocity().get().in(MetersPerSecond),
                constants.constraints.maxAcceleration().get().in(MetersPerSecondPerSecond)));

    feedforward =
        new ElevatorFeedforward(
            constants.slot0Gains.kS().get(),
            constants.slot0Gains.kG().get(),
            constants.slot0Gains.kV().get(),
            constants.slot0Gains.kA().get());

    appliedVolts = 0;
    isClosedLoop = true;

    this.constants = constants;
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

    inputs.appliedVolts = new double[constants.elevatorParameters.NUM_MOTORS()];
    inputs.supplyCurrentAmps = new double[constants.elevatorParameters.NUM_MOTORS()];
    inputs.torqueCurrentAmps = new double[constants.elevatorParameters.NUM_MOTORS()];
    inputs.temperatureCelsius = new double[constants.elevatorParameters.NUM_MOTORS()];

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
  public void setPositionGoal(double positionMeters, GainSlot slot) {
    setPositionGoal(positionMeters);
  }

  @Override
  public void setSlot(GainSlot slot) {
    switch (slot) {
      case ZERO:
        feedback.setPID(constants.slot0Gains.kP().get(), 0.0, constants.slot0Gains.kD().get());
        break;
      case ONE:
        feedback.setPID(constants.slot1Gains.kP().get(), 0.0, constants.slot1Gains.kD().get());
        break;
      case TWO:
        feedback.setPID(constants.slot2Gains.kP().get(), 0.0, constants.slot2Gains.kD().get());
        break;
    }
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
  public void updateGains(
      double kP, double kD, double kS, double kV, double kA, double kG, GainSlot slot) {
    feedback.setPID(kP, 0, kD);
    feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
  }

  @Override
  public void updateConstraints(
      double maxAcceleration, double cruisingVelocity, double goalTolerance) {
    feedback.setConstraints(new TrapezoidProfile.Constraints(cruisingVelocity, maxAcceleration));
    feedback.setTolerance(goalTolerance);
  }
}
