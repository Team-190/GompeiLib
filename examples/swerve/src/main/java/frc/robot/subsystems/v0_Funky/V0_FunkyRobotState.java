package frc.robot.subsystems.v0_Funky;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.team190.gompeilib.core.state.localization.FieldZone;
import edu.wpi.team190.gompeilib.core.state.localization.Localization;
import java.util.HashSet;
import java.util.List;

public class V0_FunkyRobotState {
  private static AprilTagFieldLayout fieldLayout;
  private static List<FieldZone> fieldZones;
  private static Localization localization;

  static {
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    FieldZone globalZone = new FieldZone(new HashSet<>(fieldLayout.getTags()));

    fieldZones = List.of(globalZone);

    localization =
        new Localization(
            fieldZones, V0_FunkyConstants.DRIVE_CONSTANTS.DRIVE_CONFIG.kinematics(), 2);
  }

  public static void periodic() {}

  public static void resetPose(Pose2d pose) {
    localization.resetPose(pose);
  }

  public static Rotation2d getHeading() {
    if (localization.getEstimatedPose(fieldZones.get(0)).isEmpty()) return Rotation2d.kZero;
    return localization.getEstimatedPose(fieldZones.get(0)).get().getRotation();
  }

  public static Pose2d getGlobalPose() {
    return localization.getEstimatedPose(fieldZones.get(0)).get();
  }
}
