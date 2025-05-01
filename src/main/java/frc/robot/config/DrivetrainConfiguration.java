package frc.robot.config;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.*;

public class DrivetrainConfiguration {
  private final String CANbusName;
  private final boolean supportsPro;
  private final Distance wheelRadius;
  private final Translation2d[] moduleLocations;
  private final boolean discretizeChassisSpeeds;
  private final LinearVelocity maxChassisSpeed;
  private final LinearAcceleration maxChassisAccel;
  private final AngularVelocity maxChassisAngularSpeed;
  private final AngularAcceleration maxChassisAngularAccel;
  private final SwerveDriveKinematics kinematics;
  private final GyroConfiguration gyroConfiguration;
  private final SwerveModuleConfiguration[] swerveModuleConfigurations;

  private DrivetrainConfiguration(Builder builder) {
    this.CANbusName = builder.CANbusName;
    this.supportsPro = builder.supportsPro;
    this.wheelRadius = builder.wheelRadius;
    this.moduleLocations = builder.moduleLocations;
    this.discretizeChassisSpeeds = builder.discretizeChassisSpeeds;
    this.maxChassisSpeed = builder.maxChassisSpeed;
    this.maxChassisAccel = builder.maxChassisAccel;
    this.maxChassisAngularSpeed = builder.maxChassisAngularSpeed;
    this.maxChassisAngularAccel = builder.maxChassisAngularAccel;
    this.kinematics = builder.kinematics;
    this.gyroConfiguration = builder.gyroConfiguration;
    this.swerveModuleConfigurations = builder.swerveModuleConfigurations;
  }

  public static class Builder {
    private String CANbusName;
    private boolean supportsPro;
    private Distance wheelRadius;
    private Translation2d[] moduleLocations;
    private boolean discretizeChassisSpeeds;
    private LinearVelocity maxChassisSpeed;
    private LinearAcceleration maxChassisAccel;
    private AngularVelocity maxChassisAngularSpeed;
    private AngularAcceleration maxChassisAngularAccel;
    private SwerveDriveKinematics kinematics;
    private GyroConfiguration gyroConfiguration;
    private SwerveModuleConfiguration[] swerveModuleConfigurations;

    public Builder setCANbusName(String CANbusName) {
      this.CANbusName = CANbusName;
      return this;
    }

    public Builder setWheelRadius(Distance wheelRadius) {
      this.wheelRadius = wheelRadius;
      return this;
    }

    public Builder setModuleLocations(Translation2d[] moduleLocations) {
      this.moduleLocations = moduleLocations;
      return this;
    }

    public Builder setDiscretizeChassisSpeeds(boolean discretizeChassisSpeeds) {
      this.discretizeChassisSpeeds = discretizeChassisSpeeds;
      return this;
    }

    public Builder setMaxChassisSpeed(LinearVelocity maxChassisSpeed) {
      this.maxChassisSpeed = maxChassisSpeed;
      return this;
    }

    public Builder setMaxChassisAccel(LinearAcceleration maxChassisAccel) {
      this.maxChassisAccel = maxChassisAccel;
      return this;
    }

    public Builder setMaxChassisAngularSpeed(AngularVelocity maxChassisAngularSpeed) {
      this.maxChassisAngularSpeed = maxChassisAngularSpeed;
      return this;
    }

    public Builder setMaxChassisAngularAccel(AngularAcceleration maxChassisAngularAccel) {
      this.maxChassisAngularAccel = maxChassisAngularAccel;
      return this;
    }

    public Builder setKinematics(SwerveDriveKinematics kinematics) {
      this.kinematics = kinematics;
      return this;
    }

    public Builder setGyroConfiguration(GyroConfiguration gyroConfiguration) {
      this.gyroConfiguration = gyroConfiguration;
      return this;
    }

    public Builder setSwerveModuleConfigurations(
        SwerveModuleConfiguration[] swerveModuleConfigurations) {
      this.swerveModuleConfigurations = swerveModuleConfigurations;
      return this;
    }

    public DrivetrainConfiguration build() {
      this.supportsPro = new CANBus(this.CANbusName).isNetworkFD();
      return new DrivetrainConfiguration(this);
    }
  }

  public String getCANbusName() {
    return CANbusName;
  }

  public boolean supportsPro() {
    return supportsPro;
  }

  public Distance getWheelRadius() {
    return wheelRadius;
  }

  public Translation2d[] getModuleLocations() {
    return moduleLocations;
  }

  public boolean isDiscretizeChassisSpeeds() {
    return discretizeChassisSpeeds;
  }

  public LinearVelocity getMaxChassisSpeed() {
    return maxChassisSpeed;
  }

  public LinearAcceleration getMaxChassisAccel() {
    return maxChassisAccel;
  }

  public AngularVelocity getMaxChassisAngularSpeed() {
    return maxChassisAngularSpeed;
  }

  public AngularAcceleration getMaxChassisAngularAccel() {
    return maxChassisAngularAccel;
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public GyroConfiguration getGyroConfiguration() {
    return gyroConfiguration;
  }

  public SwerveModuleConfiguration[] getSwerveModuleConfigurations() {
    return swerveModuleConfigurations;
  }
}
