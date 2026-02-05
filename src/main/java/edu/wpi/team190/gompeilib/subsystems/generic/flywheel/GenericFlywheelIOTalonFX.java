package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.PhoenixUtil;
import java.util.ArrayList;
import java.util.Arrays;

public class GenericFlywheelIOTalonFX implements GenericFlywheelIO {
  protected final TalonFX talonFX;
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
  private final TorqueCurrentFOC torqueCurrentFOCRequest;
  private final VelocityVoltage velocityControlRequest;
  private final MotionMagicVelocityTorqueCurrentFOC velocityTorqueCurrentRequest;

  protected GenericFlywheelConstants constants;

  public GenericFlywheelIOTalonFX(GenericFlywheelConstants constants) {
    talonFX = new TalonFX(constants.LEADER_CAN_ID);
    followerTalonFX =
        new TalonFX
            [constants.ALIGNED_FOLLOWER_CAN_IDS.length + constants.OPPOSED_FOLLOWER_CAN_IDS.length];

    talonFXConfiguration = new TalonFXConfiguration();

    talonFXConfiguration.MotorOutput.withInverted(constants.LEADER_INVERSION);

    talonFXConfiguration
        .CurrentLimits
        .withSupplyCurrentLimit(constants.CURRENT_LIMIT)
        .withSupplyCurrentLimitEnable(true);
    talonFXConfiguration.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    talonFXConfiguration
        .Slot0
        .withKP(constants.GAINS.kP().getAsDouble())
        .withKD(constants.GAINS.kD().getAsDouble())
        .withKS(constants.GAINS.kS().getAsDouble())
        .withKV(constants.GAINS.kV().getAsDouble())
        .withKA(constants.GAINS.kA().getAsDouble());

    talonFXConfiguration.Feedback.SensorToMechanismRatio = constants.GEAR_RATIO;

    talonFXConfiguration.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(
                AngularAcceleration.ofRelativeUnits(
                    constants.CONSTRAINTS.maxAccelerationRadiansPerSecondSquared().get(),
                    RotationsPerSecondPerSecond))
            .withMotionMagicCruiseVelocity(
                AngularVelocity.ofRelativeUnits(
                    constants.CONSTRAINTS.cruisingVelocityRadiansPerSecond().get(),
                    RotationsPerSecond));

    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(talonFXConfiguration, 0.25));

    final int[] indexHolder = {0}; // mutable index for array insertion

    // CCW followers
    Arrays.stream(constants.ALIGNED_FOLLOWER_CAN_IDS)
        .forEach(
            id -> {
              TalonFX follower = new TalonFX(id, talonFX.getNetwork());
              followerTalonFX[indexHolder[0]++] = follower;

              PhoenixUtil.tryUntilOk(
                  5, () -> follower.getConfigurator().apply(talonFXConfiguration, 0.25));

              follower.setControl(new Follower(talonFX.getDeviceID(), MotorAlignmentValue.Aligned));
            });

    // CW followers
    Arrays.stream(constants.OPPOSED_FOLLOWER_CAN_IDS)
        .forEach(
            id -> {
              TalonFX follower = new TalonFX(id, talonFX.getNetwork());
              followerTalonFX[indexHolder[0]++] = follower;

              PhoenixUtil.tryUntilOk(
                  5, () -> follower.getConfigurator().apply(talonFXConfiguration, 0.25));

              follower.setControl(new Follower(talonFX.getDeviceID(), MotorAlignmentValue.Opposed));
            });

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

    for (TalonFX follower : followerTalonFX) {
      appliedVolts.add(follower.getMotorVoltage());
      supplyCurrentAmps.add(follower.getSupplyCurrent());
      torqueCurrentAmps.add(follower.getTorqueCurrent());
      temperatureCelsius.add(follower.getDeviceTemp());
    }

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
    torqueCurrentFOCRequest = new TorqueCurrentFOC(0.0);

    velocityControlRequest = new VelocityVoltage(0);
    velocityTorqueCurrentRequest = new MotionMagicVelocityTorqueCurrentFOC(0.0);

    this.constants = constants;
  }

  @Override
  public void updateInputs(GenericFlywheelIOInputs inputs) {
    inputs.positionRadians = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.velocityRadiansPerSecond =
        Units.rotationsToRadians(velocityRotationsPerSecond.getValueAsDouble());

    inputs.appliedVolts = new double[appliedVolts.size()];
    inputs.supplyCurrentAmps = new double[supplyCurrentAmps.size()];
    inputs.torqueCurrentAmps = new double[torqueCurrentAmps.size()];
    inputs.temperatureCelsius = new double[temperatureCelsius.size()];

    for (int i = 0; i <= followerTalonFX.length; i++) {
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
  public void setAmps(double amps) {
    talonFX.setControl(torqueCurrentFOCRequest.withOutput(amps));
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
  public void setVelocityTorque(double velocityRadiansPerSecond) {
    velocityGoalRadiansPerSecond = velocityRadiansPerSecond;
    talonFX.setControl(
        velocityTorqueCurrentRequest.withVelocity(
            Units.radiansToRotations(velocityGoalRadiansPerSecond)));
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
      double maxAccelerationRadiansPerSecondSquared,
      double cruisingVelocity,
      double goalToleranceRadiansPerSecond) {
    talonFXConfiguration
        .MotionMagic
        .withMotionMagicAcceleration(
            Units.radiansToRotations(maxAccelerationRadiansPerSecondSquared))
        .withMotionMagicCruiseVelocity(
            AngularVelocity.ofRelativeUnits(cruisingVelocity, RotationsPerSecond));
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
