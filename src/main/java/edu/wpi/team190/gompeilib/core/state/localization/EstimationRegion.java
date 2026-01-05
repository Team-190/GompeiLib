package edu.wpi.team190.gompeilib.core.state.localization;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.team190.gompeilib.core.utility.GeometryUtil;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionTxTyObservation;
import java.util.Objects;
import java.util.Set;
import java.util.stream.Collectors;
import lombok.Getter;

public class EstimationRegion {
  @Getter private final Set<AprilTag> aprilTags;
  @Getter private final Set<Integer> tagIds;
  private final SwerveDrivePoseEstimator poseEstimator;

  public EstimationRegion(Set<AprilTag> aprilTags, SwerveDriveKinematics kinematics) {
    this.aprilTags = aprilTags;

    tagIds = aprilTags.stream().map(aprilTag -> aprilTag.ID).collect(Collectors.toSet());

    SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
    for (int i = 0; i < swerveModulePositions.length; i++) {
      swerveModulePositions[i] = new SwerveModulePosition();
    }

    this.poseEstimator =
        new SwerveDrivePoseEstimator(
            kinematics, Rotation2d.kZero, swerveModulePositions, Pose2d.kZero);
  }

  public void addOdometryObservation(
      double timestamp, Rotation2d heading, SwerveModulePosition[] modulePositions) {
    poseEstimator.updateWithTime(timestamp, heading, modulePositions);
  }

  public void addPoseObservation(VisionPoseObservation observation) {
    poseEstimator.addVisionMeasurement(
        observation.pose(), observation.timestamp(), observation.stddevs());
  }

  public void addTxTyObservation(
      VisionTxTyObservation observation, TimeInterpolatableBuffer<Pose2d> poseBuffer) {
    int tagId = observation.tagId();

    // Get odometry-based pose at the timestamp
    var sample = poseBuffer.getSample(observation.timestamp());
    if (sample.isEmpty()) return;

    // Average tx and ty over four corners
    double tx = 0.0;
    double ty = 0.0;
    for (int j = 0; j < 4; j++) {
      tx += observation.tx()[j];
      ty += observation.ty()[j];
    }
    tx /= 4.0;
    ty /= 4.0;

    Pose3d cameraPose = observation.cameraPose();

    // Project 3D distance onto horizontal plane
    double distance2d =
        observation.distance()
            * Math.cos(-cameraPose.getRotation().getY() - ty); // pitch + tag vertical angle

    // Compute rotation from camera to tag
    Rotation2d camToTagRotation =
        sample
            .get()
            .getRotation()
            .plus(cameraPose.toPose2d().getRotation().plus(Rotation2d.fromRadians(-tx)));

    Pose2d tagPose2d =
        Objects.requireNonNull(
                aprilTags.stream().filter(tag -> tag.ID == tagId).findFirst().orElse(null))
            .pose
            .toPose2d();
    if (tagPose2d == null) return;

    // Compute camera position in field frame
    Translation2d fieldToCameraTranslation =
        new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
            .transformBy(GeometryUtil.toTransform2d(distance2d, 0.0))
            .getTranslation();

    // Compute robot poses
    Pose2d robotPose =
        new Pose2d(
                fieldToCameraTranslation,
                sample.get().getRotation().plus(cameraPose.toPose2d().getRotation()))
            .transformBy(new Transform2d(cameraPose.toPose2d(), Pose2d.kZero));

    // Use odometry rotation only
    robotPose = new Pose2d(robotPose.getTranslation(), sample.get().getRotation());

    double xystdev = 0.1 * Math.pow(observation.distance(), 1.2);

    poseEstimator.addVisionMeasurement(
        robotPose, observation.timestamp(), VecBuilder.fill(xystdev, xystdev, Double.POSITIVE_INFINITY));
  }

  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }
}
