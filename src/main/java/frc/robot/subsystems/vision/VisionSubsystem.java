package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.config.CameraConfiguration;
import frc.robot.config.VisionConfiguration;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
  private final VisionConsumer consumer;
  private VisionIO[] io = new VisionIO[0];
  private final VisionIOInputsAutoLogged[] inputs;
  private final Alert[] disconnectedAlerts;
  private VisionConfiguration constants;

  public VisionSubsystem(
      VisionConfiguration constants, VisionConsumer visionConsumer, Supplier<Pose2d> poseSupplier) {
    this.constants = constants;
    this.consumer = visionConsumer;

    // Build VisionIO's
    CameraConfiguration[] cameraConfigs = constants.getCameraConfigurations();
    if (cameraConfigs != null) {
      VisionIO[] io = new VisionIO[cameraConfigs.length];
      for (int i = 0; i < cameraConfigs.length; i++) {
        CameraConfiguration camera = cameraConfigs[i];
        if (Robot.isReal()) {
          switch (camera.getCameraType()) {
            case LIMELIGHT -> io[i] =
                new VisionIOLimelight(camera.getName(), () -> poseSupplier.get().getRotation());
            case PHOTON_VISION -> io[i] =
                new VisionIOPhotonVision(
                    camera.getName(),
                    camera.getRobotToCamera(),
                    constants.getAprilTagFieldLayout());
          }
        } else {
          io[i] =
              new VisionIOPhotonVisionSim(
                  camera.getName(),
                  camera.getRobotToCamera(),
                  poseSupplier,
                  camera.getSimCameraProperties(),
                  constants.getAprilTagFieldLayout());
        }
      }

      this.io = io;
    }

    // Initialize inputs
    this.inputs = new VisionIOInputsAutoLogged[this.io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionIOInputsAutoLogged();
    }

    // Initialize disconnected alerts
    this.disconnectedAlerts = new Alert[this.io.length];
    for (int i = 0; i < inputs.length; i++) {
      disconnectedAlerts[i] =
          new Alert(
              "Vision camera " + cameraConfigs[i].getName() + " is disconnected.",
              Alert.AlertType.kWarning);
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  @Override
  public void periodic() {
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/" + inputs[i].name, inputs[i]);
    }

    // Initialize logging values
    List<Pose3d> allTagPoses = new LinkedList<>();
    List<Pose3d> allRobotPoses = new LinkedList<>();
    List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
    List<Pose3d> allRobotPosesRejected = new LinkedList<>();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Update disconnected alert
      disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

      // Initialize logging values
      List<Pose3d> tagPoses = new LinkedList<>();
      List<Pose3d> robotPoses = new LinkedList<>();
      List<Pose3d> robotPosesAccepted = new LinkedList<>();
      List<Pose3d> robotPosesRejected = new LinkedList<>();

      // Add tag poses
      for (int tagId : inputs[cameraIndex].tagIds) {
        var tagPose = constants.getAprilTagFieldLayout().getTagPose(tagId);
        tagPose.ifPresent(tagPoses::add);
      }

      // Loop over pose observations
      for (var observation : inputs[cameraIndex].poseObservations) {
        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity()
                        > constants.getMaxAmbiguity()) // Cannot be high ambiguity
                || Math.abs(observation.pose().getZ())
                    > constants.getMaxZError() // Must have realistic Z coordinate

                // Must be within the field boundaries
                || observation.pose().getX() < 0.0
                || observation.pose().getX() > constants.getAprilTagFieldLayout().getFieldLength()
                || observation.pose().getY() < 0.0
                || observation.pose().getY() > constants.getAprilTagFieldLayout().getFieldWidth();

        // Add pose to log
        robotPoses.add(observation.pose());
        if (rejectPose) {
          robotPosesRejected.add(observation.pose());
        } else {
          robotPosesAccepted.add(observation.pose());
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double stdDevFactor =
            Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
        double linearStdDev = constants.getLinearStdDevBaseline() * stdDevFactor;
        double angularStdDev = constants.getAngularStdDevBaseline() * stdDevFactor;
        if (observation.type() == VisionIO.PoseObservationType.MEGATAG_2) {
          linearStdDev *= constants.getLinearStdDevMegatag2Factor();
          angularStdDev *= constants.getAngularStdDevMegatag2Factor();
        }
        CameraConfiguration[] cameraConfig = constants.getCameraConfigurations();
        if (cameraIndex < cameraConfig.length) {
          var camera = cameraConfig[cameraIndex];
          linearStdDev *= camera.getStdDevFactor();
          angularStdDev *= camera.getStdDevFactor();
        }

        // Send vision observation
        consumer.accept(
            observation.pose().toPose2d(),
            observation.timestamp(),
            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }

      // Log camera data
      Logger.recordOutput(
          "Vision/" + inputs[cameraIndex].name + "/TagPoses", tagPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/" + inputs[cameraIndex].name + "/RobotPoses", robotPoses.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/" + inputs[cameraIndex].name + "/RobotPosesAccepted",
          robotPosesAccepted.toArray(new Pose3d[0]));
      Logger.recordOutput(
          "Vision/" + inputs[cameraIndex].name + "/RobotPosesRejected",
          robotPosesRejected.toArray(new Pose3d[0]));
      allTagPoses.addAll(tagPoses);
      allRobotPoses.addAll(robotPoses);
      allRobotPosesAccepted.addAll(robotPosesAccepted);
      allRobotPosesRejected.addAll(robotPosesRejected);
    }

    // Log combined data
    Logger.recordOutput("Vision/TagPoses", allTagPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/RobotPoses", allRobotPoses.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/RobotPosesAccepted", allRobotPosesAccepted.toArray(new Pose3d[0]));
    Logger.recordOutput("Vision/RobotPosesRejected", allRobotPosesRejected.toArray(new Pose3d[0]));
  }

  @FunctionalInterface
  public interface VisionConsumer {
    void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
