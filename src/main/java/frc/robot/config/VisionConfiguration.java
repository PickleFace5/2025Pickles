package frc.robot.config;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

public class VisionConfiguration {
  // April Tags
  private final AprilTagFieldLayout aprilTagLayout;

  // All camera configurations
  private final CameraConfiguration[] cameraConfigurations;

  // Max allowed ambiguity of measurements
  private final double maxAmbiguity;
  private final double maxZError;

  // Std Devs of linear and angular measurements
  private final double linearStdDevBaseline;
  private final double angularStdDevBaseline;
  private final double linearStdDevMegatag2Factor;
  private final double angularStdDevMegatag2Factor;

  private VisionConfiguration(Builder builder) {
    this.aprilTagLayout = builder.aprilTagLayout;
    this.cameraConfigurations = builder.cameraConfigurations;
    this.maxAmbiguity = builder.maxAmbiguity;
    this.maxZError = builder.maxZError;
    this.linearStdDevBaseline = builder.linearStdDevBaseline;
    this.angularStdDevBaseline = builder.angularStdDevBaseline;
    this.linearStdDevMegatag2Factor = builder.linearStdDevMegatag2Factor;
    this.angularStdDevMegatag2Factor = builder.angularStdDevMegatag2Factor;
  }

  public static class Builder {
    private AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    private CameraConfiguration[] cameraConfigurations;
    private double maxAmbiguity = 0.3;
    private double maxZError = 0.75;
    private double linearStdDevBaseline = 0.02;
    private double angularStdDevBaseline = 0.06;
    private double linearStdDevMegatag2Factor = 0.5;
    private double angularStdDevMegatag2Factor = Double.POSITIVE_INFINITY;

    public Builder() {}

    public Builder setAprilTagLayout(AprilTagFieldLayout layout) {
      this.aprilTagLayout = layout;
      return this;
    }

    public Builder setCameraConfigurations(CameraConfiguration[] cameraConfigurations) {
      this.cameraConfigurations = cameraConfigurations;
      return this;
    }

    public Builder setMaxAmbiguity(double maxAmbiguity) {
      this.maxAmbiguity = maxAmbiguity;
      return this;
    }

    public Builder setMaxZError(double maxZError) {
      this.maxZError = maxZError;
      return this;
    }

    public Builder setLinearStdDevBaseline(double linearStdDevBaseline) {
      this.linearStdDevBaseline = linearStdDevBaseline;
      return this;
    }

    public Builder setAngularStdDevBaseline(double angularStdDevBaseline) {
      this.angularStdDevBaseline = angularStdDevBaseline;
      return this;
    }

    public Builder setLinearStdDevMegatag2Factor(double linearStdDevMegatag2Factor) {
      this.linearStdDevMegatag2Factor = linearStdDevMegatag2Factor;
      return this;
    }

    public Builder setAngularStdDevMegatag2Factor(double angularStdDevMegatag2Factor) {
      this.angularStdDevMegatag2Factor = angularStdDevMegatag2Factor;
      return this;
    }

    public VisionConfiguration build() {
      return new VisionConfiguration(this);
    }
  }

  public AprilTagFieldLayout getAprilTagFieldLayout() {
    return aprilTagLayout;
  }

  public CameraConfiguration[] getCameraConfigurations() {
    return cameraConfigurations;
  }

  public double getMaxAmbiguity() {
    return maxAmbiguity;
  }

  public double getMaxZError() {
    return maxZError;
  }

  public double getLinearStdDevBaseline() {
    return linearStdDevBaseline;
  }

  public double getAngularStdDevBaseline() {
    return angularStdDevBaseline;
  }

  public double getLinearStdDevMegatag2Factor() {
    return linearStdDevMegatag2Factor;
  }

  public double getAngularStdDevMegatag2Factor() {
    return angularStdDevMegatag2Factor;
  }
}
