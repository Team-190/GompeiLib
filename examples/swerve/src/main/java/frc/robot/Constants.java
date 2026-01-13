package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.team190.gompeilib.core.GompeiLib;
import edu.wpi.team190.gompeilib.core.GompeiLib.Mode;

public final class Constants {
  public static final boolean TUNING_MODE = false;
  public static final double LOOP_PERIOD_SECONDS = 0.02;
  public static final RobotType ROBOT = RobotType.EXAMPLE_ROBOT_SIM;

  public static Mode getMode() {
    switch (ROBOT) {
      case EXAMPLE_ROBOT:
        return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;

      case EXAMPLE_ROBOT_SIM:
        return Mode.SIM;

      default:
        return Mode.REAL;
    }
  }

  public static void main(String... args) {
    if (getMode().equals(GompeiLib.Mode.SIM)) {
      System.err.println("Cannot deploy, invalid mode selected: " + ROBOT);
      System.exit(1);
    }
  }

  public enum RobotType {
    EXAMPLE_ROBOT,
    EXAMPLE_ROBOT_SIM
  }
}
