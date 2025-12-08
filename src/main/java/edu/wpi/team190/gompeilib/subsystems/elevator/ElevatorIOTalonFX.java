package edu.wpi.team190.gompeilib.subsystems.elevator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.logging.Trace;
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

    for (int i = 0; i <= constants.ELEVATOR_PARAMETERS.NUM_MOTORS(); i++) {
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
  public void setPosition(double positionMeters) {}

  @Override
  public void setPositionGoal(double positionMeters) {}

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
  public void updateConstraints(double maxAcceleration, double cruisingVelocity) {
    config.MotionMagic.withMotionMagicAcceleration(maxAcceleration)
        .withMotionMagicCruiseVelocity(cruisingVelocity);
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config));
  }
}
