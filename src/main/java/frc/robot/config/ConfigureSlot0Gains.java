package frc.robot.config;

import com.ctre.phoenix6.configs.Slot0Configs;

/** Helper class to configure the gains for the drive and steer motors. */
class ConfigureSlot0Gains extends Slot0Configs {
  public ConfigureSlot0Gains(double kP, double kI, double kD, double kS, double kV, double kA) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kS = kS;
    this.kV = kV;
    this.kA = kA;
  }
}
