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
import edu.wpi.first.units.measure.*;
import edu.wpi.team190.gompeilib.core.utility.PhoenixUtil;
import java.util.ArrayList;
import java.util.Arrays;

public class GenericFlywheelIOTalonFX {
  private final TalonFX talonFX;
  private final TalonFX[] followerTalonFX;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocityRotationsPerSecond;
  private final ArrayList<StatusSignal<Voltage>> appliedVolts;
  private final ArrayList<StatusSignal<Current>> supplyCurrentAmps;
  private final ArrayList<StatusSignal<Current>> torqueCurrentAmps;
  private final ArrayList<StatusSignal<Temperature>> temperatureCelcius;
  private double goalRadiansPerSecond;
  private final StatusSignal<Double> velocitySetpointRotationsPerSecond;
  private final StatusSignal<Double> velocityErrorRotationsPerSecond;

  private StatusSignal<?>[] statusSignals;

  private final TalonFXConfiguration talonFXConfiguration;

  private final NeutralOut neutralControlRequest;
  private final VoltageOut voltageControlRequest;
  private final VelocityVoltage velocityControlRequest;

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
        .withKV(constants.GAINS.kV().getAsDouble());
    talonFXConfiguration.MotorOutput.withInverted(constants.INVERSION);

    talonFX.getConfigurator().apply(talonFXConfiguration);
    for (TalonFX follower : followerTalonFX) {
      follower.getConfigurator().apply(talonFXConfiguration);
    }
    for (TalonFX follower : followerTalonFX) {
      PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(talonFXConfiguration));
      for (int i = 0; i < constants.CAN_IDS.length; i++) {
        if (Arrays.asList(constants.COUNTERCLOCKWISE_CAN_IDS).contains(follower.getDeviceID())) {
          follower.setControl(new Follower(talonFX.getDeviceID(), MotorAlignmentValue.Opposed));
        } else {
          follower.setControl(new Follower(talonFX.getDeviceID(), MotorAlignmentValue.Aligned));
        }
      }
    }

    positionRotations = talonFX.getPosition();
    velocityRotationsPerSecond = talonFX.getVelocity();

    appliedVolts = new ArrayList<>();
    supplyCurrentAmps = new ArrayList<>();
    torqueCurrentAmps = new ArrayList<>();
    temperatureCelcius = new ArrayList<>();

    appliedVolts.add(talonFX.getMotorVoltage());
    supplyCurrentAmps.add(talonFX.getSupplyCurrent());
    torqueCurrentAmps.add(talonFX.getTorqueCurrent());
    temperatureCelcius.add(talonFX.getDeviceTemp());

    goalRadiansPerSecond = 0.0;

    velocitySetpointRotationsPerSecond = talonFX.getClosedLoopReference();
    velocityErrorRotationsPerSecond = talonFX.getClosedLoopError();

    var signalsList = new ArrayList<StatusSignal<?>>();

    signalsList.add(positionRotations);
    signalsList.add(velocityRotationsPerSecond);
    signalsList.addAll(appliedVolts);
    signalsList.addAll(supplyCurrentAmps);
    signalsList.addAll(torqueCurrentAmps);
    signalsList.addAll(temperatureCelcius);

    statusSignals = new StatusSignal[signalsList.size()];

    for (int i = 0; i < signalsList.size(); i++) {
      statusSignals[i] = signalsList.get(i);
    }

    BaseStatusSignal.setUpdateFrequencyForAll(50, statusSignals); // TODO: Make frequency a variable

    talonFX.optimizeBusUtilization();
    for (TalonFX follower : followerTalonFX) {
      follower.optimizeBusUtilization();
    }

    neutralControlRequest = new NeutralOut();
    voltageControlRequest = new VoltageOut(0.0);

    velocityControlRequest = new VelocityVoltage(0);
  }
}
