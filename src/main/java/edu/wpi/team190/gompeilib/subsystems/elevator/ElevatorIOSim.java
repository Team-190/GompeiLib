package edu.wpi.team190.gompeilib.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.phoenix.GainSlot;
import java.util.Arrays;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim sim;

  private Voltage appliedVolts;
  private boolean isClosedLoop;

  private final ProfiledPIDController feedback;
  private ElevatorFeedforward feedforward;

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

    appliedVolts = Volts.of(0.0);
    isClosedLoop = true;

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

    this.constants = constants;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    if (isClosedLoop) {
      appliedVolts =
          Volts.of(feedback.calculate(sim.getPositionMeters())
              + feedforward.calculate((feedback.getSetpoint().velocity)));
    }

    appliedVolts = Volts.of(MathUtil.clamp(appliedVolts.in(Volts), -12, 12));

    sim.setInputVoltage(appliedVolts.in(Volts));
    sim.update(GompeiLib.getLoopPeriod());

    inputs.position = Meters.of(sim.getPositionMeters());
    inputs.velocity = MetersPerSecond.of(sim.getVelocityMetersPerSecond());
    inputs.acceleration =
        MetersPerSecondPerSecond.of(-1.0); // TODO: Replace with calculation based on velocity

    inputs.appliedVolts = new Voltage[constants.elevatorParameters.NUM_MOTORS()];
    inputs.supplyCurrentAmps = new Current[constants.elevatorParameters.NUM_MOTORS()];
    inputs.torqueCurrentAmps = new Current[constants.elevatorParameters.NUM_MOTORS()];
    inputs.temperatureCelsius = new Temperature[constants.elevatorParameters.NUM_MOTORS()];

    Arrays.fill(inputs.appliedVolts, appliedVolts);
    Arrays.fill(inputs.supplyCurrentAmps, Amps.of(sim.getCurrentDrawAmps()));
    Arrays.fill(inputs.torqueCurrentAmps, Amps.of(sim.getCurrentDrawAmps()));
    Arrays.fill(inputs.temperatureCelsius, Celsius.of(0.0));

    inputs.positionGoalMeters = Meters.of(feedback.getGoal().position);
    inputs.positionSetpointMeters = Meters.of(feedback.getSetpoint().position);
    inputs.positionErrorMeters = Meters.of(feedback.getPositionError());
  }

  @Override
  public void setVoltageGoal(Voltage volts) {
    isClosedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void setPositionGoal(Distance position) {
    feedback.setGoal(position.in(Meters));
    isClosedLoop = true;
  }

  @Override
  public boolean atVoltageGoal(Voltage voltageReference) {
    return appliedVolts.isNear(voltageReference, Millivolt.of(500));
  }

  @Override
  public boolean atPositionGoal(Distance positionReference) {
    return Meters.of(sim.getPositionMeters()).isNear(positionReference, constants.constraints.goalTolerance().get());
  }

  @Override
  public void setPosition(Distance position) {
    sim.setState(position.in(Meters), sim.getVelocityMetersPerSecond());
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
  public void updateGains(
      double kP, double kD, double kS, double kV, double kA, double kG, GainSlot slot) {
    feedback.setPID(kP, 0, kD);
    feedforward = new ElevatorFeedforward(kS, kG, kV, kA);
  }

  @Override
  public void updateConstraints(
          LinearAcceleration maxAcceleration, LinearVelocity maxVelocity, Distance goalTolerance) {
    feedback.setConstraints(new TrapezoidProfile.Constraints(maxVelocity.in(MetersPerSecond), maxAcceleration.in(MetersPerSecondPerSecond)));
    feedback.setTolerance(goalTolerance.in(Meters));
  }
}
