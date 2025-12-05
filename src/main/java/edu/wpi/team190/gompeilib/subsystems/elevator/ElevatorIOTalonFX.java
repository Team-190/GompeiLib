 package edu.wpi.team190.gompeilib.subsystems.elevator;

 import com.ctre.phoenix6.StatusSignal;
 import com.ctre.phoenix6.configs.TalonFXConfiguration;
 import com.ctre.phoenix6.controls.MotionMagicVoltage;
 import com.ctre.phoenix6.controls.VoltageOut;
 import com.ctre.phoenix6.hardware.TalonFX;
 import edu.wpi.first.units.measure.*;
 import java.util.ArrayList;

 public class ElevatorIOTalonFX implements ElevatorIO {

  // Core hardware components
  public final TalonFX talonFX;
  public final TalonFX[] followTalonFX;

  // Configuration
  public final TalonFXConfiguration config;

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
    // Create lead motor
    talonFX = new TalonFX(ElevatorConstants.ELEVATOR_CAN_ID);

    // Create follower motor array (define length)
    followTalonFX = new TalonFX[ElevatorConstants.ELEVATOR_PARAMETERS.NUM_MOTORS() - 1];

    // Create follower motors (populate array)
    for (int i = 0; i < ElevatorConstants.ELEVATOR_PARAMETERS.NUM_MOTORS() - 1; i++) {
      followTalonFX[i] = new TalonFX(ElevatorConstants.ELEVATOR_CAN_ID + i + 1);
    }

    config = new TalonFXConfiguration();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {}

  @Override
  public void setPosition(double positionMeters) {}

  @Override
  public void setPositionGoal(double positionMeters) {}

  @Override
  public void setVoltage(double volts) {
    talonFX.setControl(voltageRequest.withOutput(volts).withEnableFOC(true));
  }

  @Override
  public void updateGains(
      double kP, double kI, double kD, double kS, double kV, double kA, double kG) {}

  @Override
  public void updateConstraints(double maxAcceleration, double cruisingVelocity) {}
 }
