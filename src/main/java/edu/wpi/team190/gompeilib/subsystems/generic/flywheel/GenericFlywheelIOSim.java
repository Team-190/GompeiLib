package edu.wpi.team190.gompeilib.subsystems.generic.flywheel;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class GenericFlywheelIOSim {

  private DCMotorSim[] motorSims;

  private final PIDController feedback;
  private final LinearProfile profile;
  private SimpleMotorFeedforward feedforward;

  private double appliedVolts;
}
