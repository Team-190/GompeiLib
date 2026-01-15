package edu.wpi.team190.gompeilib.subsystems.arm;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.ArrayList;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.utility.GainSlot;
import edu.wpi.team190.gompeilib.core.utility.PhoenixUtil;

public class ArmIOTalonFX implements ArmIO {
    private final TalonFX talonFX;
    private final TalonFX[] followTalonFX;

    private final StatusSignal<Angle> positionRotations;
    private final StatusSignal<AngularVelocity> velocityRotationsPerSecond;
    private ArrayList<StatusSignal<Voltage>> appliedVolts;
    private ArrayList<StatusSignal<Current>> supplyCurrentAmps;
    private ArrayList<StatusSignal<Current>> torqueCurrentAmps;
    private ArrayList<StatusSignal<Temperature>> temperatureCelsius;
    private final StatusSignal<Double> positionSetpointRotations;
    private final StatusSignal<Double> positionErrorRotations;

    private StatusSignal<?>[] statusSignals;

    private final VoltageOut voltageRequest;
    private final MotionMagicVoltage positionVoltageRequest;

    private final TalonFXConfiguration config;

    private final ArmConstants constants;

    public ArmIOTalonFX(ArmConstants constants) {
        talonFX = new TalonFX(constants.ARM_CAN_ID);
        followTalonFX = new TalonFX[constants.ARM_PARAMETERS.NUM_MOTORS() - 1];

        config = new TalonFXConfiguration();

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.SupplyCurrentLimit = constants.CURRENT_LIMITS.ARM_SUPPLY_CURRENT_LIMIT();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = constants.CURRENT_LIMITS.ARM_STATOR_CURRENT_LIMIT();
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.Feedback.SensorToMechanismRatio = constants.ARM_PARAMETERS.GEAR_RATIO();

        config.Slot0
                .withKP(constants.SLOT0_GAINS.kP().get())
                .withKD(constants.SLOT0_GAINS.kD().get())
                .withKS(constants.SLOT0_GAINS.kS().get())
                .withKV(constants.SLOT0_GAINS.kV().get())
                .withKA(constants.SLOT0_GAINS.kA().get())
                .withKG(constants.SLOT0_GAINS.kG().get())
                .withGravityType(GravityTypeValue.Arm_Cosine);

        config.Slot1
                .withKP(constants.SLOT1_GAINS.kP().get())
                .withKD(constants.SLOT1_GAINS.kD().get())
                .withKS(constants.SLOT1_GAINS.kS().get())
                .withKV(constants.SLOT1_GAINS.kV().get())
                .withKA(constants.SLOT1_GAINS.kA().get())
                .withKG(constants.SLOT1_GAINS.kG().get())
                .withGravityType(GravityTypeValue.Arm_Cosine);

        config.Slot2
                .withKP(constants.SLOT2_GAINS.kP().get())
                .withKD(constants.SLOT2_GAINS.kD().get())
                .withKS(constants.SLOT2_GAINS.kS().get())
                .withKV(constants.SLOT2_GAINS.kV().get())
                .withKA(constants.SLOT2_GAINS.kA().get())
                .withKG(constants.SLOT2_GAINS.kG().get())
                .withGravityType(GravityTypeValue.Arm_Cosine);

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.ClosedLoopGeneral.ContinuousWrap = true;
        config.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicAcceleration(
                        AngularAcceleration.ofRelativeUnits(
                                constants.CONSTRAINTS.MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED().get(),
                                RotationsPerSecondPerSecond))
                .withMotionMagicCruiseVelocity(
                        AngularVelocity.ofRelativeUnits(
                                constants.CONSTRAINTS.CRUISING_VELOCITY_ROTATIONS_PER_SECOND().get(),
                                RotationsPerSecond));

        PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));

        for (TalonFX follower : followTalonFX) {
            PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(config));
            follower.setControl(
                    new Follower(
                            talonFX.getDeviceID(),
                            (follower.getDeviceID() % 2 == 1)
                                    ? MotorAlignmentValue.Aligned
                                    : MotorAlignmentValue.Opposed));
        }

        positionRotations = talonFX.getPosition();
        velocityRotationsPerSecond = talonFX.getVelocity();
        appliedVolts.add(talonFX.getMotorVoltage());
        supplyCurrentAmps.add(talonFX.getSupplyCurrent());
        torqueCurrentAmps.add(talonFX.getTorqueCurrent());
        temperatureCelsius.add(talonFX.getDeviceTemp());

        positionSetpointRotations = talonFX.getClosedLoopReference();
        positionErrorRotations = talonFX.getClosedLoopError();

        for (int i = 0; i < constants.ARM_PARAMETERS.NUM_MOTORS() - 1; i++) {
            appliedVolts.add(followTalonFX[i].getMotorVoltage());
            supplyCurrentAmps.add(followTalonFX[i].getSupplyCurrent());
            torqueCurrentAmps.add(followTalonFX[i].getTorqueCurrent());
            temperatureCelsius.add(followTalonFX[i].getDeviceTemp());
        }

        voltageRequest = new VoltageOut(0);
        positionVoltageRequest = new MotionMagicVoltage(0);

        var signalsList = new ArrayList<StatusSignal<?>>();

        signalsList.add(positionRotations);
        signalsList.add(velocityRotationsPerSecond);
        signalsList.addAll(appliedVolts);
        signalsList.addAll(supplyCurrentAmps);
        signalsList.addAll(torqueCurrentAmps);
        signalsList.addAll(temperatureCelsius);
        signalsList.add(positionSetpointRotations);
        signalsList.add(positionErrorRotations);

        statusSignals = new StatusSignal[signalsList.size()];

        for (int i = 0; i < signalsList.size(); i++) {
            statusSignals[i] = signalsList.get(i);
        }

        BaseStatusSignal.setUpdateFrequencyForAll(1 / GompeiLib.getLoopPeriod(), statusSignals);

        talonFX.optimizeBusUtilization();

        PhoenixUtil.registerSignals(false, statusSignals);

        talonFX.setPosition(0);

        this.constants = constants;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {

        inputs.position = new Rotation2d(positionRotations.getValue());
        inputs.velocityRadiansPerSecond = velocityRotationsPerSecond.getValue().in(RadiansPerSecond);

        for (int i = 0; i < constants.ARM_PARAMETERS.NUM_MOTORS(); i++) {
            inputs.appliedVolts[i] = appliedVolts.get(i).getValueAsDouble();
            inputs.supplyCurrentAmps[i] = supplyCurrentAmps.get(i).getValueAsDouble();
            inputs.torqueCurrentAmps[i] = torqueCurrentAmps.get(i).getValueAsDouble();
            inputs.temperatureCelsius[i] = temperatureCelsius.get(i).getValueAsDouble();
        }

        inputs.positionGoal = new Rotation2d(positionVoltageRequest.getPositionMeasure());
        inputs.positionSetpoint = Rotation2d.fromRotations(positionSetpointRotations.getValueAsDouble());
        inputs.positionError = Rotation2d.fromRotations(positionErrorRotations.getValueAsDouble());
    }

    @Override
    public void setVoltage(double volts) {
        talonFX.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
    }

    @Override
    public void setPosition(Rotation2d rotation) {
        talonFX.setPosition(rotation.getRotations());
    }

    @Override
    public void setPositionGoal(Rotation2d rotation) {
        talonFX.setControl(
                positionVoltageRequest.withPosition(rotation.getRotations()).withEnableFOC(true));
    }

    @Override
    public void setSlot(GainSlot slot) {
        switch (slot) {
            case ZERO:
                talonFX.setControl(positionVoltageRequest.withSlot(0));
                break;
            case ONE:
                talonFX.setControl(positionVoltageRequest.withSlot(1));
                break;
            case TWO:
            default:
                talonFX.setControl(positionVoltageRequest.withSlot(2));
                break;
        }
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

        for (int i = 0; i < constants.ARM_PARAMETERS.NUM_MOTORS() - 1; i++) {
            followTalonFX[i] = new TalonFX(constants.ARM_CAN_ID + i + 1);
        }
    }

    @Override
    public void updateConstraints(double maxAcceleration, double cruisingVelocity) {
        config.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicAcceleration(
                        AngularAcceleration.ofRelativeUnits(maxAcceleration, RotationsPerSecondPerSecond))
                .withMotionMagicCruiseVelocity(
                        AngularVelocity.ofRelativeUnits(cruisingVelocity, RotationsPerSecond));
        PhoenixUtil.tryUntilOk(5, () -> talonFX.getConfigurator().apply(config, 0.25));

        for (int i = 0; i < constants.ARM_PARAMETERS.NUM_MOTORS() - 1; i++) {
            followTalonFX[i] = new TalonFX(constants.ARM_CAN_ID + i + 1);
        }
    }
}
