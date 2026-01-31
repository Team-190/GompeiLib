// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package edu.wpi.team190.gompeilib.core.utility;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;

/**
 * A generic SysId characterization routine. Subclass this for specific units.
 *
 * @param <U> The unit type for the output (e.g., VoltageUnit, CurrentUnit)
 */
public class CustomSysIdRoutine<U extends Unit> extends SysIdRoutineLog {
  private final Config<U> config;
  private final Mechanism<U> mechanism;
  private final MutableMeasure<U, ?, ?> outputValue;
  private final Consumer<State> recordState;

  /**
   * Create a new SysId characterization routine. * @param config Configuration with strongly typed
   * measures.
   *
   * @param mechanism Mechanism interface.
   * @param initialMutable The mutable measure instance (created by the subclass).
   */
  public CustomSysIdRoutine(
      Config<U> config, Mechanism<U> mechanism, MutableMeasure<U, ?, ?> initialMutable) {
    super(mechanism.m_name);
    this.config = config;
    this.mechanism = mechanism;
    outputValue = initialMutable;
    recordState = config.m_recordState != null ? config.m_recordState : this::recordState;
  }

  /**
   * @param m_rampRate We use Measure<Unit> for ramp rate because it is U/Time.
   */
  public record Config<U extends Unit>(
      Per<U, TimeUnit> m_rampRate,
      Measure<U> m_stepOutput,
      Measure<TimeUnit> m_timeout,
      Consumer<State> m_recordState,
      U m_outputUnit) {
    public Config(
        Per<U, TimeUnit> m_rampRate,
        Measure<U> m_stepOutput,
        Measure<TimeUnit> m_timeout,
        Consumer<State> m_recordState,
        U m_outputUnit) {
      this.m_rampRate = m_rampRate;
      this.m_stepOutput = m_stepOutput;
      this.m_timeout = m_timeout != null ? m_timeout : Seconds.of(10);
      this.m_recordState = m_recordState;
      this.m_outputUnit = m_outputUnit;
    }
  }

  public static class Mechanism<U extends Unit> {
    public final Consumer<Measure<U>> m_drive;
    public final Consumer<SysIdRoutineLog> m_log;
    public final Subsystem m_subsystem;
    public final String m_name;

    public Mechanism(
        Consumer<Measure<U>> drive,
        Consumer<SysIdRoutineLog> log,
        Subsystem subsystem,
        String name) {
      m_drive = drive;
      m_log = log != null ? log : l -> {};
      m_subsystem = subsystem;
      m_name = name != null ? name : subsystem.getName();
    }

    public Mechanism(Consumer<Measure<U>> drive, Subsystem subsystem) {
      this(drive, null, subsystem, null);
    }
  }

  public enum Direction {
    kForward,
    kReverse
  }

  public Command quasistatic(Direction direction) {
    State state =
        (direction == Direction.kForward) ? State.kQuasistaticForward : State.kQuasistaticReverse;

    double outputSign = (direction == Direction.kForward) ? 1.0 : -1.0;
    Timer timer = new Timer();

    double rampRateUnitsPerSec = config.m_rampRate.magnitude();

    return mechanism
        .m_subsystem
        .runOnce(timer::restart)
        .andThen(
            mechanism.m_subsystem.run(
                () -> {
                  mechanism.m_drive.accept(
                      outputValue.mut_replace(
                          outputSign * timer.get() * rampRateUnitsPerSec, config.m_outputUnit));

                  mechanism.m_log.accept(this);
                  recordState.accept(state);
                }))
        .finallyDo(
            () -> {
              mechanism.m_drive.accept(outputValue.mut_replace(0, config.m_outputUnit));
              recordState.accept(State.kNone);
              timer.stop();
            })
        .withName("sysid-" + state + "-" + mechanism.m_name)
        .withTimeout(config.m_timeout.in(Seconds));
  }

  public Command dynamic(Direction direction) {
    double outputSign = (direction == Direction.kForward) ? 1.0 : -1.0;
    State state = (direction == Direction.kForward) ? State.kDynamicForward : State.kDynamicReverse;

    // OPTIMIZED: Pre-calculate step magnitude safely
    double stepMagnitude = config.m_stepOutput.in(config.m_outputUnit);

    return mechanism
        .m_subsystem
        .runOnce(() -> outputValue.mut_replace(stepMagnitude * outputSign, config.m_outputUnit))
        .andThen(
            mechanism.m_subsystem.run(
                () -> {
                  mechanism.m_drive.accept(outputValue);
                  mechanism.m_log.accept(this);
                  recordState.accept(state);
                }))
        .finallyDo(
            () -> {
              mechanism.m_drive.accept(outputValue.mut_replace(0, config.m_outputUnit));
              recordState.accept(State.kNone);
            })
        .withName("sysid-" + state.toString() + "-" + mechanism.m_name)
        .withTimeout(config.m_timeout.in(Seconds));
  }
}
