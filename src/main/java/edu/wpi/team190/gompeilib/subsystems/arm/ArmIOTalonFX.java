package edu.wpi.team190.gompeilib.subsystems.arm;

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

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.team190.gompeilib.core.utility.GainSlot;
import edu.wpi.team190.gompeilib.core.utility.PhoenixUtil;

public class ArmIOTalonFX implements ArmIO {
    private final TalonFX armTalonFX;
    private final TalonFX[] followTalonFX;

    private final StatusSignal<Angle> armPositionRotations;
    private final StatusSignal<AngularVelocity> armVelocityRotationsPerSecond;
    private ArrayList<StatusSignal<Voltage>> armAppliedVolts;
    private ArrayList<StatusSignal<Current>> armSupplyCurrentAmps;
    private ArrayList<StatusSignal<Current>> armTorqueCurrentAmps;
    private ArrayList<StatusSignal<Temperature>> armTemperatureCelsius;
    private final StatusSignal<Double> armPositionSetpointRotations;
    private final StatusSignal<Double> armPositionErrorRotations;

    private StatusSignal<?>[] statusSignals;

    private final VoltageOut armVoltageRequest;
    private final MotionMagicVoltage armMotionMagicRequest;

    private final TalonFXConfiguration armConfig;

    private final ArmConstants constants;

    public ArmIOTalonFX(ArmConstants constants) {
        armTalonFX = new TalonFX(constants.ARM_CAN_ID);
        followTalonFX = new TalonFX[constants.ARM_PARAMETERS.NUM_MOTORS() - 1];

        armConfig = new TalonFXConfiguration();

        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        armConfig.CurrentLimits.SupplyCurrentLimit = constants.CURRENT_LIMITS.ARM_SUPPLY_CURRENT_LIMIT();
        armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        armConfig.CurrentLimits.StatorCurrentLimit = constants.CURRENT_LIMITS.ARM_STATOR_CURRENT_LIMIT();
        armConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        armConfig.Feedback.SensorToMechanismRatio = constants.ARM_PARAMETERS.GEAR_RATIO();

        armConfig.Slot0.withKP(constants.SLOT0_GAINS.kP().get())
                .withKD(constants.SLOT0_GAINS.kD().get())
                .withKS(constants.SLOT0_GAINS.kS().get())
                .withKV(constants.SLOT0_GAINS.kV().get())
                .withKA(constants.SLOT0_GAINS.kA().get())
                .withKG(constants.SLOT0_GAINS.kG().get())
                .withGravityType(GravityTypeValue.Arm_Cosine);

        armConfig.Slot1.withKP(constants.SLOT1_GAINS.kP().get())
                .withKD(constants.SLOT1_GAINS.kD().get())
                .withKS(constants.SLOT1_GAINS.kS().get())
                .withKV(constants.SLOT1_GAINS.kV().get())
                .withKA(constants.SLOT1_GAINS.kA().get())
                .withKG(constants.SLOT1_GAINS.kG().get())
                .withGravityType(GravityTypeValue.Arm_Cosine);

        armConfig.Slot2.withKP(constants.SLOT2_GAINS.kP().get())
                .withKD(constants.SLOT2_GAINS.kD().get())
                .withKS(constants.SLOT2_GAINS.kS().get())
                .withKV(constants.SLOT2_GAINS.kV().get())
                .withKA(constants.SLOT2_GAINS.kA().get())
                .withKG(constants.SLOT2_GAINS.kG().get())
                .withGravityType(GravityTypeValue.Arm_Cosine);

        armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        armConfig.ClosedLoopGeneral.ContinuousWrap = true;
        armConfig.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicAcceleration(
                        AngularAcceleration.ofRelativeUnits(
                                constants.CONSTRAINTS
                                        .MAX_ACCELERATION_ROTATIONS_PER_SECOND_SQUARED()
                                        .get(),
                                RotationsPerSecondPerSecond))
                .withMotionMagicCruiseVelocity(
                        AngularVelocity.ofRelativeUnits(
                                constants.CONSTRAINTS
                                        .CRUISING_VELOCITY_ROTATIONS_PER_SECOND()
                                        .get(),
                                RotationsPerSecond));

        PhoenixUtil.tryUntilOk(5, () -> armTalonFX.getConfigurator().apply(armConfig, 0.25));

        for (TalonFX follower : followTalonFX) {
            PhoenixUtil.tryUntilOk(5, () -> follower.getConfigurator().apply(armConfig));
            follower.setControl(
                    new Follower(
                            armTalonFX.getDeviceID(),
                            (follower.getDeviceID() % 2 == 1)
                                    ? MotorAlignmentValue.Aligned
                                    : MotorAlignmentValue.Opposed));
        }

        armPositionRotations = armTalonFX.getPosition();
        armVelocityRotationsPerSecond = armTalonFX.getVelocity();
        armAppliedVolts.add(armTalonFX.getMotorVoltage());
        armSupplyCurrentAmps.add(armTalonFX.getSupplyCurrent());
        armTorqueCurrentAmps.add(armTalonFX.getTorqueCurrent());
        armTemperatureCelsius.add(armTalonFX.getDeviceTemp());

        armPositionSetpointRotations = armTalonFX.getClosedLoopReference();
        armPositionErrorRotations = armTalonFX.getClosedLoopError();

        for (int i = 0; i < constants.ARM_PARAMETERS.NUM_MOTORS() - 1; i++) {
            armAppliedVolts.add(followTalonFX[i].getMotorVoltage());
            armSupplyCurrentAmps.add(followTalonFX[i].getSupplyCurrent());
            armTorqueCurrentAmps.add(followTalonFX[i].getTorqueCurrent());
            armTemperatureCelsius.add(followTalonFX[i].getDeviceTemp());
        }

        armVoltageRequest = new VoltageOut(0);
        armMotionMagicRequest = new MotionMagicVoltage(0);

        var signalsList = new ArrayList<StatusSignal<?>>();

        signalsList.add(armPositionRotations);
        signalsList.add(armVelocityRotationsPerSecond);
        signalsList.addAll(armAppliedVolts);
        signalsList.addAll(armSupplyCurrentAmps);
        signalsList.addAll(armTorqueCurrentAmps);
        signalsList.addAll(armTemperatureCelsius);
        signalsList.add(armPositionSetpointRotations);
        signalsList.add(armPositionErrorRotations);

        statusSignals = new StatusSignal[signalsList.size()];

        for (int i = 0; i < signalsList.size(); i++) {
            statusSignals[i] = signalsList.get(i);
        }

        BaseStatusSignal.setUpdateFrequencyForAll(50, statusSignals);

        armTalonFX.optimizeBusUtilization();

        PhoenixUtil.registerSignals(false, statusSignals);

        armTalonFX.setPosition(0);

        this.constants = constants;
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {

        inputs.position = new Rotation2d(armPositionRotations.getValue());
        inputs.velocityRadiansPerSecond = armVelocityRotationsPerSecond.getValue().in(RadiansPerSecond);

        for (int i = 0; i < constants.ARM_PARAMETERS.NUM_MOTORS(); i++) {
            inputs.appliedVolts[i] = armAppliedVolts.get(i).getValueAsDouble();
            inputs.supplyCurrentAmps[i] = armSupplyCurrentAmps.get(i).getValueAsDouble();
            inputs.torqueCurrentAmps[i] = armTorqueCurrentAmps.get(i).getValueAsDouble();
            inputs.temperatureCelsius[i] = armTemperatureCelsius.get(i).getValueAsDouble();
        }

        inputs.positionGoal = new Rotation2d(armMotionMagicRequest.getPositionMeasure());
        inputs.positionSetpoint = Rotation2d.fromRotations(armPositionSetpointRotations.getValueAsDouble());
        inputs.positionError = Rotation2d.fromRotations(armPositionErrorRotations.getValueAsDouble());
    }

    @Override
    public void setVoltage(double volts) {
        armTalonFX.setControl(armVoltageRequest.withOutput(volts).withEnableFOC(true));
    }

    @Override
    public void setPosition(Rotation2d rotation) {
        armTalonFX.setPosition(
        rotation.getRotations() * constants.ARM_PARAMETERS.GEAR_RATIO());
    }

    @Override
    public void setPositionGoal(Rotation2d rotation) {
        armTalonFX.setControl(
                armMotionMagicRequest.withPosition(rotation.getRotations()).withEnableFOC(true));
    }

    @Override
    public void setSlot(GainSlot slot) {
        switch (slot) {
            case ZERO:
                armTalonFX.setControl(armMotionMagicRequest.withSlot(0));
                break;
            case ONE:
                armTalonFX.setControl(armMotionMagicRequest.withSlot(1));
                break;
            case TWO:
            default:
                armTalonFX.setControl(armMotionMagicRequest.withSlot(2));
                break;
        }
    }

    @Override
    public void updateGains(
            double kP, double kD, double kS, double kV, double kA, double kG, GainSlot slot) {
        switch (slot) {
            case ZERO:
                armConfig.Slot0.withKP(kP).withKD(kD).withKS(kS).withKV(kV).withKA(kA).withKG(kG);
                break;
            case ONE:
                armConfig.Slot1.withKP(kP).withKD(kD).withKS(kS).withKV(kV).withKA(kA).withKG(kG);
                break;
            case TWO:
            default:
                armConfig.Slot2.withKP(kP).withKD(kD).withKS(kS).withKV(kV).withKA(kA).withKG(kG);
                break;
        }
        PhoenixUtil.tryUntilOk(5, () -> armTalonFX.getConfigurator().apply(armConfig));
    }

    @Override
    public void updateConstraints(double maxAcceleration, double cruisingVelocity) {
        armConfig.MotionMagic = new MotionMagicConfigs()
                .withMotionMagicAcceleration(
                        AngularAcceleration.ofRelativeUnits(maxAcceleration, RotationsPerSecondPerSecond))
                .withMotionMagicCruiseVelocity(
                        AngularVelocity.ofRelativeUnits(cruisingVelocity, RotationsPerSecond));
        PhoenixUtil.tryUntilOk(5, () -> armTalonFX.getConfigurator().apply(armConfig, 0.25));
    }
}
