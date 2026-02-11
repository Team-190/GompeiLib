package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.team190.gompeilib.core.utility.phoenix.PhoenixUtil;
import java.util.ArrayList;

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
    talonFX = new TalonFX(constants.leaderCANID);
    followerTalonFX =
        new TalonFX
            [constants.alignedFollowerCANIDs.size() + constants.opposedFollowerCANIDs.size()];

    talonFXConfiguration = new TalonFXConfiguration();

    talonFXConfiguration.MotorOutput.withInverted(constants.leaderInversion);

    talonFXConfiguration
        .CurrentLimits
        .withSupplyCurrentLimit(constants.currentLimit.supplyCurrentLimit())
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(constants.currentLimit.statorCurrentLimit())
        .withStatorCurrentLimitEnable(true);
    talonFXConfiguration.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
    talonFXConfiguration
        .Slot0
        .withKP(constants.gains.kP().getAsDouble())
        .withKD(constants.gains.kD().getAsDouble())
        .withKS(constants.gains.kS().getAsDouble())
        .withKV(constants.gains.kV().getAsDouble())
        .withKA(constants.gains.kA().getAsDouble());

    talonFXConfiguration.Feedback.SensorToMechanismRatio = constants.gearRatio;

    talonFXConfiguration.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicAcceleration(
                constants.constraints.maxAcceleration().get().in(RotationsPerSecondPerSecond))
            .withMotionMagicCruiseVelocity(
                constants.constraints.maxVelocity().get().in(RotationsPerSecond));

    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(talonFXConfiguration, 0.25));

    final int[] indexHolder = {0}; // mutable index for array insertion

    constants.alignedFollowerCANIDs.forEach(
        id -> {
          TalonFX follower = new TalonFX(id, talonFX.getNetwork());
          followerTalonFX[indexHolder[0]++] = follower;

          PhoenixUtil.tryUntilOk(
              5, () -> follower.getConfigurator().apply(talonFXConfiguration, 0.25));

          follower.setControl(new Follower(talonFX.getDeviceID(), MotorAlignmentValue.Aligned));
        });

    constants.opposedFollowerCANIDs.forEach(
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

    PhoenixUtil.registerSignals(constants.canBus.isNetworkFD(), statusSignals);

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
    talonFX.setControl(voltageControlRequest.withOutput(volts).withEnableFOC(constants.enableFOC));
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
            .withEnableFOC(constants.enableFOC));
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
        <= constants.constraints.goalTolerance().get().in(Rotations);
  }

  @Override
  public void stop() {
    talonFX.setControl(neutralControlRequest);
  }
}
