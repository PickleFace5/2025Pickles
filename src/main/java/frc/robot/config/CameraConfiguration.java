package frc.robot.config;

public class CameraConfiguration {
  public enum CameraType {
    LIMELIGHT,
    PHOTON_VISION
  }

  private final String name;
  private final CameraType type;
  private int fps;
  private double fov;

  private CameraConfiguration(Builder builder) {
    this.name = builder.name;
    this.type = builder.type;
    this.fps = builder.fps;
    this.fov = builder.fov;
  }

  public static class Builder {
    private final String name;
    private final CameraType type;
    private int fps = 60;
    private double fov = 60;

    public Builder(String name, CameraType type) {
      this.name = name;
      this.type = type;
    }

    public Builder setFps(int fps) {
      this.fps = fps;
      return this;
    }

    public Builder setFov(double fov) {
      this.fov = fov;
      return this;
    }

    public CameraConfiguration build() {
      return new CameraConfiguration(this);
    }
  }
}
