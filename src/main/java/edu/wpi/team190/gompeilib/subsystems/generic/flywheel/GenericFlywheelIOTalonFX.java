package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.PhoenixUtil;
import java.util.ArrayList;

public class GenericFlywheelIOTalonFX implements GenericFlywheelIO {
  private final TalonFX talonFX;
  private final TalonFX[] followerTalonFX;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  private final ArrayList<StatusSignal<Voltage>> appliedVolts;
  private final ArrayList<StatusSignal<Current>> supplyCurrentAmps;
  private final ArrayList<StatusSignal<Current>> torqueCurrentAmps;
  private final ArrayList<StatusSignal<Temperature>> temperatureCelsius;
  private double velocityGoalRadiansPerSecond;
  private final StatusSignal<Double> velocitySetpointRotationsPerSecond;
  private final StatusSignal<Double> velocityErrorRotationsPerSecond;

  private StatusSignal<?>[] statusSignals;

  private final TalonFXConfiguration talonFXConfiguration;

  private final NeutralOut neutralControlRequest;
  private final VoltageOut voltageControlRequest;
  private final VelocityVoltage velocityControlRequest;

  GenericFlywheelConstants constants;

  public GenericFlywheelIOTalonFX(GenericFlywheelConstants constants) {
    talonFX = new TalonFX(constants.CAN_IDS[0]);
    followerTalonFX = new TalonFX[constants.CAN_IDS.length - 1];
    for (int i = 1; i < constants.CAN_IDS.length; i++) {
      followerTalonFX[i - 1] = new TalonFX(constants.CAN_IDS[i]);
    }

    talonFXConfiguration = new TalonFXConfiguration();

    talonFXConfiguration
        .CurrentLimits
        .withSupplyCurrentLimit(constants.CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true);
    talonFXConfiguration.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    talonFXConfiguration
        .Slot0
        .withKP(constants.GAINS.kP().getAsDouble())
        .withKD(constants.GAINS.kD().getAsDouble())
        .withKS(constants.GAINS.kS().getAsDouble())
        .withKV(constants.GAINS.kV().getAsDouble())
        .withKA(constants.GAINS.kA().getAsDouble());
    talonFXConfiguration.MotorOutput.withInverted(constants.INVERSION);

    talonFXConfiguration.Feedback.SensorToMechanismRatio = constants.GEAR_RATIO;

    talonFX.getConfigurator().apply(talonFXConfiguration);
    for (TalonFX follower : followerTalonFX) {
      PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(talonFXConfiguration));
      follower.setControl(
          new Follower(
              talonFX.getDeviceID(),
              (follower.getDeviceID() % 2 == 1)
                  ? MotorAlignmentValue.Aligned
                  : MotorAlignmentValue.Opposed));
    }

    positionRotations = talonFX.getPosition();
    velocityRotationsPerSecond = talonFX.getVelocity();

    appliedVolts = new ArrayList<>();
    supplyCurrentAmps = new ArrayList<>();
    torqueCurrentAmps = new ArrayList<>();
    temperatureCelsius = new ArrayList<>();

    appliedVolts.add(talonFX.getMotorVoltage());
    supplyCurrentAmps.add(talonFX.getSupplyCurrent());
    torqueCurrentAmps.add(talonFX.getTorqueCurrent());
    temperatureCelsius.add(talonFX.getDeviceTemp());

    velocityGoalRadiansPerSecond = 0.0;

    velocitySetpointRotationsPerSecond = talonFX.getClosedLoopReference();
    velocityErrorRotationsPerSecond = talonFX.getClosedLoopError();

    var signalsList = new ArrayList<StatusSignal<?>>();

    signalsList.add(positionRotations);
    signalsList.add(velocityRotationsPerSecond);
    signalsList.addAll(appliedVolts);
    signalsList.addAll(supplyCurrentAmps);
    signalsList.addAll(torqueCurrentAmps);
    signalsList.addAll(temperatureCelsius);

    statusSignals = new StatusSignal[signalsList.size()];

    for (int i = 0; i < signalsList.size(); i++) {
      statusSignals[i] = signalsList.get(i);
    }

    BaseStatusSignal.setUpdateFrequencyForAll(1 / GompeiLib.getLoopPeriod(), statusSignals);

    PhoenixUtil.registerSignals(constants.ON_CANIVORE, statusSignals);

    talonFX.optimizeBusUtilization();
    for (TalonFX follower : followerTalonFX) {
      follower.optimizeBusUtilization();
    }

    neutralControlRequest = new NeutralOut();
    voltageControlRequest = new VoltageOut(0.0);

    velocityControlRequest = new VelocityVoltage(0);

    this.constants = constants;
  }

  @Override
  public void updateInputs(GenericFlywheelIOInputs inputs) {
    inputs.positionRadians = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.velocityRadiansPerSecond =
        Units.rotationsToRadians(velocityRotationsPerSecond.getValueAsDouble());

    inputs.appliedVolts = new double[constants.CAN_IDS.length];
    inputs.supplyCurrentAmps = new double[constants.CAN_IDS.length];
    inputs.torqueCurrentAmps = new double[constants.CAN_IDS.length];
    inputs.temperatureCelsius = new double[constants.CAN_IDS.length];

    for (int i = 0; i < constants.CAN_IDS.length; i++) {
      inputs.appliedVolts[i] = appliedVolts.get(i).getValueAsDouble();
      inputs.supplyCurrentAmps[i] = supplyCurrentAmps.get(i).getValueAsDouble();
      inputs.torqueCurrentAmps[i] = torqueCurrentAmps.get(i).getValueAsDouble();
      inputs.temperatureCelsius[i] = temperatureCelsius.get(i).getValueAsDouble();
    }

    inputs.velocityGoalRadiansPerSecond = velocityGoalRadiansPerSecond;
    inputs.velocitySetpointRadiansPerSecond =
        Units.rotationsToRadians(velocitySetpointRotationsPerSecond.getValueAsDouble());
    inputs.velocityErrorRadiansPerSecond =
        Units.rotationsToRadians(velocityRotationsPerSecond.getValueAsDouble());
  }

  @Override
  public void setVoltage(double volts) {
    talonFX.setControl(voltageControlRequest.withOutput(volts).withEnableFOC(constants.ENABLE_FOC));
  }

  @Override
  public void setVelocity(double velocityRadiansPerSecond) {
    velocityGoalRadiansPerSecond = velocityRadiansPerSecond;
    talonFX.setControl(
        velocityControlRequest
            .withVelocity(Units.radiansToRotations(velocityGoalRadiansPerSecond))
            .withEnableFOC(constants.ENABLE_FOC));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    talonFXConfiguration.Slot0.withKP(kP).withKI(kI).withKD(kD);
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(talonFXConfiguration));
    for (TalonFX follower : followerTalonFX) {
      PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(talonFXConfiguration));
    }
  }

  @Override
  public void setFeedforward(double kS, double kV, double kA) {
    talonFXConfiguration.Slot0.withKS(kS).withKV(kV).withKA(kA);
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(talonFXConfiguration));
    for (TalonFX follower : followerTalonFX) {
      PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(talonFXConfiguration));
    }
  }

  @Override
  public void setProfile(
      double maxAccelerationRadiansPerSecondSquared, double goalToleranceRadiansPerSecond) {
    talonFXConfiguration.MotionMagic.withMotionMagicAcceleration(
        Units.radiansToRotations(maxAccelerationRadiansPerSecondSquared));
    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(talonFXConfiguration));
    for (TalonFX follower : followerTalonFX) {
      PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(talonFXConfiguration));
    }
  }

  @Override
  public boolean atGoal() {
    return Math.abs(velocityErrorRotationsPerSecond.getValueAsDouble())
        <= Units.radiansToRotations(constants.CONSTRAINTS.goalToleranceRadiansPerSecond().get());
  }

  @Override
  public void stop() {
    talonFX.setControl(neutralControlRequest);
  }
}
