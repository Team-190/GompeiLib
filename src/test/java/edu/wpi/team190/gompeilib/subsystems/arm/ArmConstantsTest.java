package edu.wpi.team190.gompeilib.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.team190.gompeilib.core.utility.control.Gains;
import edu.wpi.team190.gompeilib.core.utility.control.constraints.AngularPositionConstraints;
import org.junit.jupiter.api.*;

@TestMethodOrder(MethodOrderer.OrderAnnotation.class)
public class ArmConstantsTest {

  private ArmConstants.ArmParameters createParameters() {
    return ArmConstants.ArmParameters.builder()
        .withMotorConfig(DCMotor.getNEO(1))
        .withMinAngle(Rotation2d.fromDegrees(0))
        .withMaxAngle(Rotation2d.fromDegrees(180))
        .withContinuousOutput(false)
        .withNumMotors(1)
        .withGearRatio(100.0)
        .withLengthMeters(1.0)
        .withMomentOfInertia(0.5)
        .build();
  }

  private AngularPositionConstraints createConstraints() {
    return AngularPositionConstraints.fromMeasures()
        .withPrefix("test")
        .withGoalTolerance(Degrees.of(1))
        .withMaxVelocity(DegreesPerSecond.of(360))
        .withMaxAcceleration(DegreesPerSecondPerSecond.of(720))
        .build();
  }

  private ArmConstants createConstants() {
    return ArmConstants.builder()
        .withArmCANID(1)
        .withCanBus(new CANBus("rio"))
        .withArmParameters(createParameters())
        .withSlot0Gains(Gains.builder().build())
        .withConstraints(createConstraints())
        .withEnableFOC(true)
        .withInvertedValue(InvertedValue.Clockwise_Positive)
        .withVoltageOffsetStep(Units.Volts.of(1))
        .withPositionOffsetStep(Rotation2d.fromDegrees(1))
        .build();
  }

  @Test
  @Order(1)
  void testBuilderSetsValues() {
    ArmConstants constants = createConstants();

    assertEquals(1, constants.armCANID);
    assertEquals("rio", constants.canBus.getName());
    assertTrue(constants.enableFOC);
    assertEquals(InvertedValue.Clockwise_Positive, constants.invertedValue);
  }

  @Test
  @Order(2)
  void testDefaultGainsNotNull() {
    ArmConstants constants = createConstants();

    assertNotNull(constants.slot1Gains);
    assertNotNull(constants.slot2Gains);
  }

  @Test
  @Order(3)
  void testCustomSlot0Gains() {
    Gains gains =
        Gains.fromDoubles()
            .withPrefix("test")
            .withKP(1.0)
            .withKI(0.0)
            .withKD(0.0)
            .withKS(0.0)
            .withKV(0.0)
            .withKA(0.0)
            .withKG(0.0)
            .build();
    ArmConstants constants =
        ArmConstants.builder()
            .withArmCANID(1)
            .withCanBus(new CANBus("rio"))
            .withArmParameters(createParameters())
            .withSlot0Gains(gains)
            .withConstraints(createConstraints())
            .withEnableFOC(true)
            .withInvertedValue(InvertedValue.Clockwise_Positive)
            .withVoltageOffsetStep(Units.Volts.of(1))
            .withPositionOffsetStep(Rotation2d.fromDegrees(1))
            .build();

    assertEquals(1.0, constants.slot0Gains.getKP());
  }

  @Test
  @Order(4)
  void testArmParametersValues() {
    ArmConstants.ArmParameters params = createParameters();

    assertEquals(1, params.numMotors());
    assertEquals(100.0, params.gearRatio());
    assertEquals(1.0, params.lengthMeters());
    assertFalse(params.continuousOutput());
  }

  @Test
  @Order(5)
  void testVoltageAndPositionOffsets() {
    ArmConstants constants = createConstants();

    Voltage voltage = constants.voltageOffsetStep;
    Rotation2d rotation = constants.positionOffsetStep;

    assertEquals(1.0, voltage.in(Units.Volts));
    assertEquals(1.0, rotation.getDegrees());
  }

  @Test
  @Order(6)
  void testNullSafety() {
    assertThrows(
        NullPointerException.class,
        () -> {
          ArmConstants.builder()
              .withArmCANID(null) // required field
              .build();
        });
  }
}
