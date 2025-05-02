package frc.robot.config;

import static frc.robot.util.MacAddressUtil.getMACAddress;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.MacAddressUtil;

public enum RobotIdentity {
  LEVIATHAN,
  METAL_MELODY,
  LARRY,
  SIMULATION;

  public static RobotIdentity getIdentity() {
    if (!Robot.isReal()) {
      return SIMULATION;
    } else {
      String mac = getMACAddress();
      if (!mac.isEmpty()) {
        switch (mac) {
          case MacAddressUtil.LEVIATHAN_MAC -> {
            return LEVIATHAN;
          }
          case MacAddressUtil.METAL_MELODY_MAC -> {
            return METAL_MELODY;
          }
          case MacAddressUtil.LARRY_MAC -> {
            return LARRY;
          }
          default -> throw new IllegalArgumentException(
              "Robot with MAC address '" + mac + "' not supported!");
        }
      }

      return LEVIATHAN;
    }
  }

  public static Constants.Mode getMode() {
    return switch (getIdentity()) {
      case LEVIATHAN, METAL_MELODY, LARRY -> Robot.isReal()
          ? Constants.Mode.REAL
          : Constants.Mode.REPLAY;
      case SIMULATION -> Constants.Mode.SIM;
    };
  }
}
