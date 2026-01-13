package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.team190.gompeilib.core.state.localization.FieldZone;
import edu.wpi.team190.gompeilib.core.state.localization.Localization;
import java.util.List;
import java.util.Set;

public class ExampleRobotRobotState {
  public static AprilTagFieldLayout fieldLayout;
  public static List<FieldZone> fieldZones;
  public static Localization localization;

  static {
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
    fieldZones =
        List.of(
            new FieldZone(
                Set.of(
                    fieldLayout.getTags().get(1),
                    fieldLayout.getTags().get(2),
                    fieldLayout.getTags().get(3))));
    localization =
        new Localization(
            fieldZones, ExampleRobotConstants.DRIVE_CONSTANTS.DRIVE_CONFIG.kinematics(), 2);
  }

  public static void periodic() {}

  public static void resetPose(Pose2d pose) {
    localization.resetPose(pose);
  }

  public static Rotation2d getHeading() {
    return localization.getEstimatedPose(fieldZones.get(0)).get().getRotation();
  }
}
