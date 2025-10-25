package frc.robot.config;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.drivers.CanDeviceId;

public class SwerveModuleConfiguration {
  public enum ClosedLoopOutputType {
    Voltage,
    TorqueCurrentFOC
  }

  public enum SteerFeedbackType {
    RemoteCANcoder,
    FusedCANcoder,
    SyncCANcoder,
  }

  private final String name;

  // CAN IDs
  private final CanDeviceId driveCanDeviceId;
  private final CanDeviceId steerCanDeviceId;
  private final CanDeviceId encoderCanDeviceId;

  // Encoder config
  private final Angle encoderOffset;
  private final boolean encoderInverted;

  // Gear ratios
  private final double driveGearRatio;
  private final double steerGearRatio;
  private final double couplingGearRatio;


  // Other physical properties
  private final Distance wheelRadius;
  private final LinearVelocity speedAt12Volts;

  // Drive + steer configs
  private final Slot0Configs driveMotorGains;
  private final Slot0Configs steerMotorGains;
  private final ClosedLoopOutputType driveClosedLoopOutput;
  private final ClosedLoopOutputType steerClosedLoopOutput;
  private final Current driveMotorSupplyCurrent;
  private final Current driveMotorStatorCurrent;
  private final boolean enableDriveMotorSupplyCurrent;
  private final boolean enableDriveMotorStatorCurrent;
  private final boolean driveMotorInverted;
  private final Current steerMotorSupplyCurrent;
  private final Current steerMotorStatorCurrent;
  private final boolean enableSteerMotorSupplyCurrent;
  private final boolean enableSteerMotorStatorCurrent;
  private final boolean steerMotorInverted;

  // Module settings
  private final boolean applyCosineCompensation;
  private final boolean appleCouplingCompensation;

  private final SteerFeedbackType feedbackSource;

  private SwerveModuleConfiguration(Builder builder) {
    this.name = builder.name;
    this.driveCanDeviceId = builder.driveCanDeviceId;
    this.steerCanDeviceId = builder.steerCanDeviceId;
    this.encoderCanDeviceId = builder.encoderCanDeviceId;
    this.encoderOffset = builder.encoderOffset;
    this.encoderInverted = builder.encoderInverted;
    this.driveGearRatio = builder.driveGearRatio;
    this.steerGearRatio = builder.steerGearRatio;
    this.couplingGearRatio = builder.couplingGearRatio;
    this.wheelRadius = builder.wheelRadius;
    this.driveMotorGains = builder.driveMotorGains;
    this.steerMotorGains = builder.steerMotorGains;
    this.driveClosedLoopOutput = builder.driveClosedLoopOutput;
    this.steerClosedLoopOutput = builder.steerClosedLoopOutput;
    this.driveMotorSupplyCurrent = builder.driveMotorSupplyCurrent;
    this.driveMotorStatorCurrent = builder.driveMotorStatorCurrent;
    this.enableDriveMotorSupplyCurrent = builder.enableDriveMotorSupplyCurrent;
    this.enableDriveMotorStatorCurrent = builder.enableDriveMotorStatorCurrent;
    this.driveMotorInverted = builder.driveMotorInverted;
    this.steerMotorSupplyCurrent = builder.steerMotorSupplyCurrent;
    this.steerMotorStatorCurrent = builder.steerMotorStatorCurrent;
    this.enableSteerMotorSupplyCurrent = builder.enableSteerMotorSupplyCurrent;
    this.enableSteerMotorStatorCurrent = builder.enableSteerMotorStatorCurrent;
    this.steerMotorInverted = builder.steerMotorInverted;
    this.speedAt12Volts = builder.speedAt12Volts;
    this.applyCosineCompensation = builder.applyCosineCompensation;
    this.appleCouplingCompensation = builder.appleCouplingCompensation;
    this.feedbackSource = builder.feedbackSource;
  }

  public static class Builder {
    private String name = "Unnamed Module";
    private CanDeviceId driveCanDeviceId = new CanDeviceId(-1);
    private CanDeviceId steerCanDeviceId = new CanDeviceId(-1);
    private CanDeviceId encoderCanDeviceId = new CanDeviceId(-1);
    private Angle encoderOffset = Units.Rotations.of(0);
    private boolean encoderInverted = false;
    private double driveGearRatio = 1.0;
    private double steerGearRatio = 1.0;
    private double couplingGearRatio = 1.0;
    private Distance wheelRadius = Units.Inches.of(2);
    private Slot0Configs driveMotorGains = new Slot0Configs();
    private Slot0Configs steerMotorGains = new Slot0Configs();
    private ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private Current driveMotorSupplyCurrent = Units.Amps.of(40);
    private Current driveMotorStatorCurrent = Units.Amps.of(80);
    private boolean enableDriveMotorSupplyCurrent = true;
    private boolean enableDriveMotorStatorCurrent = true;
    private boolean driveMotorInverted = false;
    private Current steerMotorSupplyCurrent = Units.Amps.of(40);
    private Current steerMotorStatorCurrent = Units.Amps.of(80);
    private boolean enableSteerMotorSupplyCurrent = true;
    private boolean enableSteerMotorStatorCurrent = true;
    private boolean steerMotorInverted = false;
    private LinearVelocity speedAt12Volts = Units.MetersPerSecond.of(4.0);
    private boolean applyCosineCompensation = true;
    private boolean appleCouplingCompensation = true;
    private SteerFeedbackType feedbackSource = SteerFeedbackType.RemoteCANcoder;

    public Builder() {}

    public Builder(SwerveModuleConfiguration config) {
      this.name = config.name;
      this.driveCanDeviceId = config.driveCanDeviceId;
      this.steerCanDeviceId = config.steerCanDeviceId;
      this.encoderCanDeviceId = config.encoderCanDeviceId;
      this.encoderOffset = config.encoderOffset;
      this.encoderInverted = config.encoderInverted;
      this.driveGearRatio = config.driveGearRatio;
      this.steerGearRatio = config.steerGearRatio;
      this.couplingGearRatio = config.couplingGearRatio;
      this.wheelRadius = config.wheelRadius;
      this.driveMotorGains = config.driveMotorGains;
      this.steerMotorGains = config.steerMotorGains;
      this.driveClosedLoopOutput = config.driveClosedLoopOutput;
      this.steerClosedLoopOutput = config.steerClosedLoopOutput;
      this.driveMotorSupplyCurrent = config.driveMotorSupplyCurrent;
      this.driveMotorStatorCurrent = config.driveMotorStatorCurrent;
      this.enableDriveMotorSupplyCurrent = config.enableDriveMotorSupplyCurrent;
      this.enableDriveMotorStatorCurrent = config.enableDriveMotorStatorCurrent;
      this.driveMotorInverted = config.driveMotorInverted;
      this.steerMotorSupplyCurrent = config.steerMotorSupplyCurrent;
      this.steerMotorStatorCurrent = config.steerMotorStatorCurrent;
      this.enableSteerMotorSupplyCurrent = config.enableSteerMotorSupplyCurrent;
      this.enableSteerMotorStatorCurrent = config.enableSteerMotorStatorCurrent;
      this.steerMotorInverted = config.steerMotorInverted;
      this.speedAt12Volts = config.speedAt12Volts;
      this.applyCosineCompensation = config.applyCosineCompensation;
      this.appleCouplingCompensation = config.appleCouplingCompensation;
      this.feedbackSource = config.feedbackSource;
    }

    public Builder setName(String name) {
      this.name = name;
      return this;
    }

    public Builder setDriveCanDeviceId(CanDeviceId id) {
      this.driveCanDeviceId = id;
      return this;
    }

    public Builder setSteerCanDeviceId(CanDeviceId id) {
      this.steerCanDeviceId = id;
      return this;
    }

    public Builder setEncoderCanDeviceId(CanDeviceId id) {
      this.encoderCanDeviceId = id;
      return this;
    }

    public Builder setEncoderOffset(Angle encoderOffset) {
      this.encoderOffset = encoderOffset;
      return this;
    }

    public Builder setEncoderInverted(boolean encoderInverted) {
      this.encoderInverted = encoderInverted;
      return this;
    }

    public Builder setDriveGearRatio(double ratio) {
      this.driveGearRatio = ratio;
      return this;
    }

    public Builder setSteerGearRatio(double ratio) {
      this.steerGearRatio = ratio;
      return this;
    }

    public Builder setCouplingGearRatio(double ratio) {
      this.couplingGearRatio = ratio;
      return this;
    }

    public Builder setWheelRadius(Distance radius) {
      this.wheelRadius = radius;
      return this;
    }

    public Builder setDriveMotorGains(Slot0Configs gains) {
      this.driveMotorGains = gains;
      return this;
    }

    public Builder setSteerMotorGains(Slot0Configs gains) {
      this.steerMotorGains = gains;
      return this;
    }

    public Builder setDriveClosedLoopOutput(ClosedLoopOutputType closedLoopOutputType) {
      this.driveClosedLoopOutput = closedLoopOutputType;
      return this;
    }

    public Builder setSteerClosedLoopOutput(ClosedLoopOutputType closedLoopOutputType) {
      this.steerClosedLoopOutput = closedLoopOutputType;
      return this;
    }

    public Builder setDriveMotorSupplyCurrent(Current current) {
      this.driveMotorSupplyCurrent = current;
      return this;
    }

    public Builder setDriveMotorStatorCurrent(Current current) {
      this.driveMotorStatorCurrent = current;
      return this;
    }

    public Builder setEnableDriveMotorSupplyCurrent(boolean enabled) {
      this.enableDriveMotorSupplyCurrent = enabled;
      return this;
    }

    public Builder setEnableDriveMotorStatorCurrent(boolean enabled) {
      this.enableDriveMotorStatorCurrent = enabled;
      return this;
    }

    public Builder setDriveMotorInverted(boolean inverted) {
      this.driveMotorInverted = inverted;
      return this;
    }

    public Builder setSteerMotorSupplyCurrent(Current current) {
      this.steerMotorSupplyCurrent = current;
      return this;
    }

    public Builder setSteerMotorStatorCurrent(Current current) {
      this.steerMotorStatorCurrent = current;
      return this;
    }

    public Builder setEnableSteerMotorSupplyCurrent(boolean enabled) {
      this.enableSteerMotorSupplyCurrent = enabled;
      return this;
    }

    public Builder setEnableSteerMotorStatorCurrent(boolean enabled) {
      this.enableSteerMotorStatorCurrent = enabled;
      return this;
    }

    public Builder setSteerMotorInverted(boolean inverted) {
      this.steerMotorInverted = inverted;
      return this;
    }

    public Builder setSpeedAt12Volts(LinearVelocity speed) {
      this.speedAt12Volts = speed;
      return this;
    }

    public Builder setApplyCosineCompensation(boolean apply) {
      this.applyCosineCompensation = apply;
      return this;
    }

    public Builder setApplyCouplingCompensation(boolean apply) {
      this.appleCouplingCompensation = apply;
      return this;
    }

    public Builder setFeedbackSource(SteerFeedbackType source) {
      this.feedbackSource = source;
      return this;
    }

    public SwerveModuleConfiguration build() {
      return new SwerveModuleConfiguration(this);
    }
  }

  public String getName() {
    return name;
  }

  public CanDeviceId getDriveCanDeviceId() {
    return driveCanDeviceId;
  }

  public CanDeviceId getSteerCanDeviceId() {
    return steerCanDeviceId;
  }

  public CanDeviceId getEncoderCanDeviceId() {
    return encoderCanDeviceId;
  }

  public Angle getEncoderOffset() {
    return encoderOffset;
  }

  public boolean isEncoderInverted() {
    return encoderInverted;
  }

  public double getDriveGearRatio() {
    return driveGearRatio;
  }

  public double getSteerGearRatio() {
    return steerGearRatio;
  }

  public double getCouplingGearRatio() {
    return couplingGearRatio;
  }

  public Distance getWheelRadius() {
    return wheelRadius;
  }

  public Slot0Configs getDriveMotorGains() {
    return driveMotorGains;
  }

  public Slot0Configs getSteerMotorGains() {
    return steerMotorGains;
  }

  public ClosedLoopOutputType getDriveClosedLoopOutput() {
    return driveClosedLoopOutput;
  }

  public ClosedLoopOutputType getSteerClosedLoopOutput() {
    return steerClosedLoopOutput;
  }

  public Current getDriveMotorSupplyCurrent() {
    return driveMotorSupplyCurrent;
  }

  public Current getDriveMotorStatorCurrent() {
    return driveMotorStatorCurrent;
  }

  public boolean enableDriveMotorSupplyCurrent() {
    return enableDriveMotorSupplyCurrent;
  }

  public boolean enableDriveMotorStatorCurrent() {
    return enableDriveMotorStatorCurrent;
  }

  public boolean isDriveMotorInverted() {
    return driveMotorInverted;
  }

  public Current getSteerMotorSupplyCurrent() {
    return steerMotorSupplyCurrent;
  }

  public Current getSteerMotorStatorCurrent() {
    return steerMotorStatorCurrent;
  }

  public boolean enableSteerMotorSupplyCurrent() {
    return enableSteerMotorSupplyCurrent;
  }

  public boolean enableSteerMotorStatorCurrent() {
    return enableSteerMotorStatorCurrent;
  }

  public boolean isSteerMotorInverted() {
    return steerMotorInverted;
  }

  public LinearVelocity getSpeedAt12Volts() {
    return speedAt12Volts;
  }

  public boolean applyCosineCompensation() {
    return applyCosineCompensation;
  }

  public boolean applyCouplingCompensation() {
    return appleCouplingCompensation;
  }

  public SteerFeedbackType getFeedbackSource() {
    return feedbackSource;
  }
}
