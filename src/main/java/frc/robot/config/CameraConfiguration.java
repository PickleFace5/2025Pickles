package frc.robot.config;

import edu.wpi.first.math.geometry.Transform3d;
import org.photonvision.simulation.SimCameraProperties;

public class CameraConfiguration {
  public enum CameraType {
    LIMELIGHT,
    PHOTON_VISION
  }

  // Name of camera
  private final String name;

  // Limelight or PhotonVision
  private final CameraType type;

  // Position of camera relative to the center of the robot (from the ground)
  private final Transform3d robotToCamera;

  // Camera specs
  private final int fps;
  private final double fov;

  // Confidence of vision measurements (0-1, 1=100%)
  private final double stdDevFactor;

  // Simulation properties
  private final SimCameraProperties simCameraProperties;

  private CameraConfiguration(Builder builder) {
    this.name = builder.name;
    this.type = builder.type;
    this.robotToCamera = builder.robotToCamera;
    this.fps = builder.fps;
    this.fov = builder.fov;
    this.stdDevFactor = builder.stdDevFactor;
    this.simCameraProperties = builder.simCameraProperties;
  }

  public static class Builder {
    private final String name;
    private final CameraType type;
    private final Transform3d robotToCamera;
    private int fps = 60;
    private double fov = 60;
    private double stdDevFactor = 1.0;
    private SimCameraProperties simCameraProperties = new SimCameraProperties();

    public Builder(String name, CameraType type, Transform3d robotToCamera) {
      this.name = name;
      this.type = type;
      this.robotToCamera = robotToCamera;
    }

    public Builder setFps(int fps) {
      this.fps = fps;
      return this;
    }

    public Builder setFov(double fov) {
      this.fov = fov;
      return this;
    }

    public Builder setStdDevFactor(double factor) {
      this.stdDevFactor = factor;
      return this;
    }

    public Builder setSimCameraProperties(SimCameraProperties simCameraProperties) {
      this.simCameraProperties = simCameraProperties;
      return this;
    }

    public CameraConfiguration build() {
      return new CameraConfiguration(this);
    }
  }

  public String getName() {
    return name;
  }

  public CameraType getCameraType() {
    return type;
  }

  public Transform3d getRobotToCamera() {
    return robotToCamera;
  }

  public int getFPS() {
    return fps;
  }

  public double getFOV() {
    return fov;
  }

  public double getStdDevFactor() {
    return stdDevFactor;
  }

  public SimCameraProperties getSimCameraProperties() {
    return simCameraProperties;
  }
}
