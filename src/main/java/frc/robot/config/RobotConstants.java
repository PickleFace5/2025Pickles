package frc.robot.config;

import com.pathplanner.lib.config.RobotConfig;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public interface RobotConstants {
  DrivetrainConfiguration getDrivetrainConfiguration();

  DriveTrainSimulationConfig getMapleSimConfig();

  RobotConfig getPathPlannerConfig();

  static RobotConstants getRobotConstants(RobotIdentity robot) {
    return switch (robot) {
      case LEVIATHAN, SIMULATION -> new Leviathan();
      case METAL_MELODY -> new Leviathan();
      case LARRY -> new Leviathan();
    };
  }
}
