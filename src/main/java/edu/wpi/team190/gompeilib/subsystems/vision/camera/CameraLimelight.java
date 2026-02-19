package edu.wpi.team190.gompeilib.subsystems.vision.camera;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.team190.gompeilib.core.utility.LimelightHelpers;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants;
import edu.wpi.team190.gompeilib.subsystems.vision.VisionConstants.LimelightConfig;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionPoseObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.data.VisionSingleTxTyObservation;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIO;
import edu.wpi.team190.gompeilib.subsystems.vision.io.CameraIOLimelight;
import edu.wpi.team190.gompeilib.subsystems.vision.io.LimelightIOInputsAutoLogged;
import java.util.*;
import java.util.function.Consumer;
import java.util.function.LongSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import lombok.Getter;
import org.littletonrobotics.junction.Logger;

public class CameraLimelight extends Camera {
  private final LimelightIOInputsAutoLogged inputs;
  private final CameraIOLimelight io;

  private final LimelightConfig config;
  @Getter private final String name;

  private final Supplier<Rotation2d> headingSupplier;
  private final LongSupplier timestampSupplier;
  private final DoubleArrayPublisher headingPublisher;

  @Getter private final List<Pose3d> allTagPoses;

  public CameraLimelight(
      CameraIOLimelight io,
      LimelightConfig config,
      Supplier<Rotation2d> headingSupplier,
      LongSupplier timestampSupplier,
      List<Consumer<List<VisionPoseObservation>>> poseObservers,
      List<Consumer<List<VisionSingleTxTyObservation>>> singleTxTyObservers) {
    super(config.key(), poseObservers, new ArrayList<>(), singleTxTyObservers);

    inputs = new LimelightIOInputsAutoLogged();
    this.io = io;

    this.config = config;
    this.name = "limelight-" + this.config.key();

    this.headingSupplier = headingSupplier;
    this.timestampSupplier = timestampSupplier;
    this.headingPublisher =
        NetworkTableInstance.getDefault()
            .getTable(this.name)
            .getDoubleArrayTopic("robot_orientation_set")
            .publish();

    allTagPoses = new ArrayList<>();

    currentCameraPose =
        new Pose3d(
            config.robotToCameraTransform().getTranslation(),
            config.robotToCameraTransform().getRotation());

    LimelightHelpers.setCameraPose_RobotSpace(
        name,
        currentCameraPose.getX(),
        currentCameraPose.getY(),
        currentCameraPose.getZ(),
        currentCameraPose.getRotation().getMeasureX().in(Degrees),
        currentCameraPose.getRotation().getMeasureY().in(Degrees),
        currentCameraPose.getRotation().getMeasureZ().in(Degrees));
  }

  @Override
  public void periodic() {
    poseObservationList.clear();
    multiTxTyObservationList.clear();
    singleTxTyObservationList.clear();

    this.headingPublisher.set(
        new double[] {this.headingSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0},
        timestampSupplier.getAsLong());

    io.updateInputs(inputs);
    Logger.processInputs("Vision/Cameras/" + this.name, inputs);

    allTagPoses.clear();

    double xyStdDev = config.megatagXYStdev();
    double thetaStdev = config.metatagThetaStdev();

    if (inputs.mt1PoseEstimate.tagCount() != 0) {
      xyStdDev =
          config.megatagXYStdev()
              * Math.pow(
                  inputs.mt1PoseEstimate.avgTagDist(), VisionConstants.XY_STDEV_DISTANCE_EXPONENT)
              / Math.pow(
                  inputs.mt1PoseEstimate.tagCount(), VisionConstants.XY_STDEV_TAG_COUNT_EXPONENT);
      thetaStdev =
          inputs.mt1PoseEstimate.tagCount() > 1
              ? config.metatagThetaStdev()
                  * Math.pow(
                      inputs.mt1PoseEstimate.avgTagDist(),
                      VisionConstants.XY_STDEV_DISTANCE_EXPONENT)
                  / Math.pow(
                      inputs.mt1PoseEstimate.tagCount(),
                      VisionConstants.XY_STDEV_TAG_COUNT_EXPONENT)
              : Double.POSITIVE_INFINITY;
    }

    if (inputs.mt1PoseEstimate.tagCount() != 0) {
      poseObservationList.add(
          new VisionPoseObservation(
              inputs.mt1PoseEstimate.pose(),
              Arrays.stream(inputs.mt1PoseEstimate.rawFiducials())
                  .map(CameraIO.RawFiducial::id)
                  .collect(Collectors.toSet()),
              inputs.mt1PoseEstimate.timestampSeconds(),
              VecBuilder.fill(xyStdDev, xyStdDev, thetaStdev)));
    }

    if (inputs.mt1PoseEstimate.tagCount() != 0) {
      xyStdDev =
              config.megatag2XYStdev()
                      * Math.pow(
                      inputs.mt1PoseEstimate.avgTagDist(), VisionConstants.XY_STDEV_DISTANCE_EXPONENT)
                      / Math.pow(
                      inputs.mt1PoseEstimate.tagCount(), VisionConstants.XY_STDEV_TAG_COUNT_EXPONENT);
      thetaStdev = Double.POSITIVE_INFINITY;
    }

    if (inputs.mt2PoseEstimate.tagCount() != 0) {
      poseObservationList.add(
          new VisionPoseObservation(
              inputs.mt2PoseEstimate.pose(),
              Arrays.stream(inputs.mt2PoseEstimate.rawFiducials())
                  .map(CameraIO.RawFiducial::id)
                  .collect(Collectors.toSet()),
              inputs.mt2PoseEstimate.timestampSeconds(),
              VecBuilder.fill(xyStdDev, xyStdDev, thetaStdev)));
    }

    if (inputs.rawFiducials.length != 0) {
      Arrays.stream(inputs.rawFiducials)
          .forEach(
              fiducial ->
                  singleTxTyObservationList.add(
                      new VisionSingleTxTyObservation(
                          fiducial.id(),
                          fiducial.txnc(),
                          fiducial.tync(),
                          fiducial.distToCamera(),
                          inputs.mt1PoseEstimate.timestampSeconds(),
                          currentCameraPose)));
    }

    super.sendObservers();
  }
}
