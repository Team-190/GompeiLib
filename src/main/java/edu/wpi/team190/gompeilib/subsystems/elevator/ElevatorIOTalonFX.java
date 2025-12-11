package edu.wpi.team190.gompeilib.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.team190.gompeilib.core.utility.GainSlot;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.utility.PhoenixUtil;
import java.util.ArrayList;

public class ElevatorIOTalonFX implements ElevatorIO {

  // Core hardware components
  public final TalonFX talonFX;
  public final TalonFX[] followTalonFX;

  // Configuration
  public final TalonFXConfiguration config;
  public final ElevatorConstants constants;

  // Sensor inputs
  private StatusSignal<Angle> positionRotations;
  private StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  private ArrayList<StatusSignal<Voltage>> appliedVolts;
  private ArrayList<StatusSignal<Current>> supplyCurrentAmps;
  private ArrayList<StatusSignal<Current>> torqueCurrentAmps;
  private ArrayList<StatusSignal<Temperature>> temperatureCelsius;

  public double positionGoalMeters;

  private StatusSignal<Double> positionSetpointRotations;
  private StatusSignal<Double> positionErrorRotations;

  private StatusSignal<?>[] statusSignals;

  private MotionMagicVoltage positionVoltageRequest;
  private VoltageOut voltageRequest;

  public ElevatorIOTalonFX(ElevatorConstants constants) {

    this.constants = constants;

    // Create lead motor
    talonFX = new TalonFX(constants.ELEVATOR_CAN_ID);

    // Create follower motor array (define length)
    followTalonFX = new TalonFX[constants.ELEVATOR_PARAMETERS.NUM_MOTORS() - 1];

    // Create follower motors (populate array)
    for (int i = 0; i < constants.ELEVATOR_PARAMETERS.NUM_MOTORS() - 1; i++) {
      followTalonFX[i] = new TalonFX(constants.ELEVATOR_CAN_ID + i + 1);
    }

    config = new TalonFXConfiguration();
    config.Slot0.withKP(constants.SLOT0_GAINS.kP().get())
        .withKD(constants.SLOT0_GAINS.kD().get())
        .withKS(constants.SLOT0_GAINS.kS().get())
        .withKV(constants.SLOT0_GAINS.kV().get())
        .withKA(constants.SLOT0_GAINS.kA().get())
        .withKG(constants.SLOT0_GAINS.kG().get())
        .withGravityType(GravityTypeValue.Elevator_Static);

    config.Slot1.withKP(constants.SLOT1_GAINS.kP().get())
        .withKD(constants.SLOT1_GAINS.kD().get())
        .withKS(constants.SLOT1_GAINS.kS().get())
        .withKV(constants.SLOT1_GAINS.kV().get())
        .withKA(constants.SLOT1_GAINS.kA().get())
        .withKG(constants.SLOT1_GAINS.kG().get())
        .withGravityType(GravityTypeValue.Elevator_Static);

    config.Slot2.withKP(constants.SLOT2_GAINS.kP().get())
        .withKD(constants.SLOT2_GAINS.kD().get())
        .withKS(constants.SLOT2_GAINS.kS().get())
        .withKV(constants.SLOT2_GAINS.kV().get())
        .withKA(constants.SLOT2_GAINS.kA().get())
        .withKG(constants.SLOT2_GAINS.kG().get())
        .withGravityType(GravityTypeValue.Elevator_Static);

    config.CurrentLimits.withSupplyCurrentLimit(constants.ELEVATOR_SUPPLY_CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(constants.ELEVATOR_STATOR_CURRENT_LIMIT)
        .withStatorCurrentLimitEnable(true);

    config.SoftwareLimitSwitch.withForwardSoftLimitThreshold(
            constants.ELEVATOR_PARAMETERS.MAX_HEIGHT_METERS()
                * constants.ELEVATOR_GEAR_RATIO
                / (2 * Math.PI * constants.DRUM_RADIUS))
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(
            constants.ELEVATOR_PARAMETERS.MIN_HEIGHT_METERS()
                * constants.ELEVATOR_GEAR_RATIO
                / (2 * Math.PI * constants.DRUM_RADIUS))
        .withReverseSoftLimitEnable(true);

    config.MotionMagic.withMotionMagicAcceleration(
            constants.CONSTRAINTS.maxAccelerationMetersPerSecondSquared().getAsDouble()
                * constants.ELEVATOR_GEAR_RATIO
                / (2 * Math.PI * constants.DRUM_RADIUS))
        .withMotionMagicCruiseVelocity(
            constants.CONSTRAINTS.cruisingVelocityMetersPerSecond().getAsDouble()
                * constants.ELEVATOR_GEAR_RATIO
                / (2 * Math.PI * constants.DRUM_RADIUS));

    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config));

    for (TalonFX follower : followTalonFX) {
      PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(config));
      follower.setControl(new Follower(talonFX.getDeviceID(), !(follower.getDeviceID() % 2 == 0)));
    }

    appliedVolts = new ArrayList<>();
    supplyCurrentAmps = new ArrayList<>();
    torqueCurrentAmps = new ArrayList<>();
    temperatureCelsius = new ArrayList<>();

    positionRotations = talonFX.getPosition();
    velocityRotationsPerSecond = talonFX.getVelocity();
    appliedVolts.add(talonFX.getMotorVoltage());
    supplyCurrentAmps.add(talonFX.getSupplyCurrent());
    torqueCurrentAmps.add(talonFX.getTorqueCurrent());
    temperatureCelsius.add(talonFX.getDeviceTemp());
    positionGoalMeters = 0.0;
    positionSetpointRotations = talonFX.getClosedLoopReference();
    positionErrorRotations = talonFX.getClosedLoopError();

    for (int i = 0; i < constants.ELEVATOR_PARAMETERS.NUM_MOTORS() - 1; i++) {
      appliedVolts.add(followTalonFX[i].getMotorVoltage());
      supplyCurrentAmps.add(followTalonFX[i].getSupplyCurrent());
      torqueCurrentAmps.add(followTalonFX[i].getTorqueCurrent());
      temperatureCelsius.add(followTalonFX[i].getDeviceTemp());
    }

    var signalsList = new ArrayList<StatusSignal<?>>();

    signalsList.add(positionRotations);
    signalsList.add(velocityRotationsPerSecond);
    signalsList.add(positionSetpointRotations);
    signalsList.add(positionErrorRotations);
    signalsList.addAll(appliedVolts);
    signalsList.addAll(supplyCurrentAmps);
    signalsList.addAll(torqueCurrentAmps);
    signalsList.addAll(temperatureCelsius);

    statusSignals = new StatusSignal[signalsList.size()];

    for (int i = 0; i < signalsList.size(); i++) {
      statusSignals[i] = signalsList.get(i);
    }

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, statusSignals); // TODO: make the frequency a variable

    talonFX.optimizeBusUtilization();
    for (TalonFX follower : followTalonFX) {
      follower.optimizeBusUtilization();
    }

    positionVoltageRequest = new MotionMagicVoltage(0.0);
    voltageRequest = new VoltageOut(0.0);

    PhoenixUtil.registerSignals(constants.ON_CANIVORE, statusSignals);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    inputs.positionMeters =
        positionRotations.getValueAsDouble()
            * 2
            * Math.PI
            * constants.DRUM_RADIUS
            / constants.ELEVATOR_GEAR_RATIO;
    inputs.velocityMetersPerSecond =
        velocityRotationsPerSecond.getValueAsDouble()
            * 2
            * Math.PI
            * constants.DRUM_RADIUS
            / constants.ELEVATOR_GEAR_RATIO;
    inputs.accelerationMetersPerSecondSquared = -1; // TODO: Replace with an actual value

    for (int i = 0; i < constants.ELEVATOR_PARAMETERS.NUM_MOTORS(); i++) {
      inputs.appliedVolts[i] = appliedVolts.get(i).getValueAsDouble();
      inputs.supplyCurrentAmps[i] = supplyCurrentAmps.get(i).getValueAsDouble();
      inputs.torqueCurrentAmps[i] = torqueCurrentAmps.get(i).getValueAsDouble();
      inputs.temperatureCelsius[i] = temperatureCelsius.get(i).getValueAsDouble();
    }
    inputs.positionGoalMeters = positionGoalMeters;
    inputs.positionSetpointMeters =
        positionSetpointRotations.getValueAsDouble()
            * 2
            * Math.PI
            * constants.DRUM_RADIUS
            / constants.ELEVATOR_GEAR_RATIO;
    inputs.positionErrorMeters =
        positionErrorRotations.getValueAsDouble()
            * 2
            * Math.PI
            * constants.DRUM_RADIUS
            / constants.ELEVATOR_GEAR_RATIO;
  }

  @Override
  public void setPosition(double positionMeters) {
    talonFX.setPosition(
        positionMeters / (2 * Math.PI * constants.DRUM_RADIUS) * constants.ELEVATOR_GEAR_RATIO);
  }

  @Override
  public void setPositionGoal(double positionMeters) {
    positionGoalMeters = positionMeters;
    talonFX.setControl(
        positionVoltageRequest
            .withPosition(
                positionMeters
                    / (2 * Math.PI * constants.DRUM_RADIUS)
                    * constants.ELEVATOR_GEAR_RATIO)
            .withSlot(0));
  }

  @Override
  public void setVoltage(double volts) {
    talonFX.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void updateGains(double kP, double kD, double kS, double kV, double kA, double kG) {
    config.Slot0.withKP(kP).withKD(kD).withKS(kS).withKV(kV).withKA(kA).withKG(kG);
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config));
  }

  @Override
  public void updateGains(
          double kP, double kD, double kS, double kV, double kA, double kG, GainSlot slot) {
      switch (slot) {
          case ZERO:
              config.Slot0.withKP(kP).withKD(kD).withKS(kS).withKV(kV).withKA(kA).withKG(kG);
              break;
          case ONE:
              config.Slot1.withKP(kP).withKD(kD).withKS(kS).withKV(kV).withKA(kA).withKG(kG);
              break;
          case TWO:
          default:
              config.Slot2.withKP(kP).withKD(kD).withKS(kS).withKV(kV).withKA(kA).withKG(kG);
              break;
      }
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config));
  }

  @Override
  public void updateConstraints(double maxAcceleration, double cruisingVelocity) {
    config.MotionMagic.withMotionMagicAcceleration(maxAcceleration)
        .withMotionMagicCruiseVelocity(cruisingVelocity);
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config));
  }
}
