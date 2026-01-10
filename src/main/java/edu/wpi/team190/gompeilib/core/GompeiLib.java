package edu.wpi.team190.gompeilib.core;

/** Main class for the GompeiLib library. Must be initialized by calling init() in robotInit(). */
public final class GompeiLib {

  // Define an enum within the library to avoid depending on the robot code's enum.
  public static enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  private static boolean initialized = false;
  private static Mode currentMode;
  private static boolean tuningMode;
  private static double loopPeriod;

  /**
   * Initializes the GompeiLib library with global constants from the robot project. This method
   * should be called once in robotInit().
   *
   * @param mode The current robot mode (REAL, SIM, or REPLAY).
   * @param isTuning Whether tuning mode is enabled.
   * @param loopPeriodSecs The robot's main loop period in seconds.
   */
  public static void init(Mode mode, boolean isTuning, double loopPeriodSecs) {
    if (initialized) {
      System.err.println("GompeiLib has already been initialized!");
      return;
    }

    currentMode = mode;
    tuningMode = isTuning;
    loopPeriod = loopPeriodSecs;

    initialized = true;
  }

  /** Throws an exception if the library has not been initialized. */
  private static void checkInitialized() {
    if (!initialized) {
      throw new IllegalStateException("GompeiLib.init() must be called before using GompeiLib.");
    }
  }

  // Public getters for other classes in your library to use.
  public static Mode getMode() {
    checkInitialized();
    return currentMode;
  }

  public static boolean isTuning() {
    checkInitialized();
    return tuningMode;
  }

  public static double getLoopPeriod() {
    checkInitialized();
    return loopPeriod;
  }
}
