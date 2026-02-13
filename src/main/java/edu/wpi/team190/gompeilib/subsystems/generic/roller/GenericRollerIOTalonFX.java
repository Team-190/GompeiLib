package edu.wpi.team190.gompeilib.subsystems.generic.roller;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.PhoenixUtil;

public class GenericRollerIOTalonFX implements GenericRollerIO {
  protected final TalonFX talonFX;

  private final TalonFXConfiguration config;

  private final StatusSignal<Angle> positionRotations;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVoltage;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> torqueCurrent;
  private final StatusSignal<Temperature> temperature;

  private final VoltageOut voltageRequest;

  protected GenericRollerConstants constants;

  public GenericRollerIOTalonFX(GenericRollerConstants constants) {
    talonFX = new TalonFX(constants.rollerCANID, constants.canBus);

    config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = constants.neutralMode;
    config.CurrentLimits.SupplyCurrentLimit = constants.supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));

    positionRotations = talonFX.getPosition();
    velocity = talonFX.getVelocity();
    appliedVoltage = talonFX.getMotorVoltage();
    supplyCurrent = talonFX.getSupplyCurrent();
    torqueCurrent = talonFX.getTorqueCurrent();
    temperature = talonFX.getDeviceTemp();

    voltageRequest = new VoltageOut(0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        1 / GompeiLib.getLoopPeriod(),
        positionRotations,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        temperature);

    talonFX.optimizeBusUtilization();

    PhoenixUtil.registerSignals(
        constants.canBus.isNetworkFD(),
        positionRotations,
        velocity,
        appliedVoltage,
        supplyCurrent,
        torqueCurrent,
        temperature);

    this.constants = constants;
  }

  @Override
  public void updateInputs(GenericRollerIOInputs inputs) {
    inputs.position = Rotation2d.fromRotations(positionRotations.getValueAsDouble());
    inputs.velocity = velocity.getValue();
    inputs.appliedVolts = appliedVoltage.getValue();
    inputs.supplyCurrent = supplyCurrent.getValue();
    inputs.torqueCurrent = torqueCurrent.getValue();
    inputs.temperature = temperature.getValue();
  }

  @Override
  public void setVoltage(double volts) {
    talonFX.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }
}
