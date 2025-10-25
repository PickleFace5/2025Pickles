// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.config.DrivetrainConfiguration;
import frc.robot.config.RobotConstants;
import frc.robot.config.RobotIdentity;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.gyro.GyroIO;
import frc.robot.subsystems.drive.gyro.GyroIOSim;
import frc.robot.subsystems.drive.module.ModuleIO;
import frc.robot.subsystems.drive.module.ModuleIOSim;
import frc.robot.subsystems.drive.module.ModuleIOTalonFX;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final DriveSubsystem driveSubsystem;
  public SwerveDriveSimulation driveSimulation = null;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {

    // Get robot constants from MAC Address.
    RobotConstants constants = RobotConstants.getRobotConstants(RobotIdentity.getIdentity());
    DrivetrainConfiguration swerveConfig = constants.getDrivetrainConfiguration();

    switch (RobotIdentity.getMode()) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        driveSubsystem =
            new DriveSubsystem(
                constants,
                swerveConfig
                    .getGyroConfiguration()
                    .getGyroType()
                    .createIO(constants.getDrivetrainConfiguration().getGyroConfiguration()),
                new ModuleIOTalonFX(swerveConfig.getSwerveModuleConfigurations()[0]),
                new ModuleIOTalonFX(swerveConfig.getSwerveModuleConfigurations()[1]),
                new ModuleIOTalonFX(swerveConfig.getSwerveModuleConfigurations()[2]),
                new ModuleIOTalonFX(swerveConfig.getSwerveModuleConfigurations()[3]),
                (robotPose) -> {});
        new VisionSubsystem(
            driveSubsystem,
            new VisionIOLimelight(VisionConstants.frontCamera, driveSubsystem::getRotation),
            new VisionIOLimelight(VisionConstants.frontLeftCamera, driveSubsystem::getRotation),
            new VisionIOLimelight(VisionConstants.frontRightCamera, driveSubsystem::getRotation));
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        driveSimulation =
            new SwerveDriveSimulation(
                constants.getMapleSimConfig(), new Pose2d(3, 3, Rotation2d.kZero));
        SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation);
        driveSubsystem =
            new DriveSubsystem(
                constants,
                new GyroIOSim(driveSimulation.getGyroSimulation()) {},
                new ModuleIOSim(
                    swerveConfig.getSwerveModuleConfigurations()[0],
                    driveSimulation.getModules()[0]) {},
                new ModuleIOSim(
                    swerveConfig.getSwerveModuleConfigurations()[1],
                    driveSimulation.getModules()[1]) {},
                new ModuleIOSim(
                    swerveConfig.getSwerveModuleConfigurations()[2],
                    driveSimulation.getModules()[2]) {},
                new ModuleIOSim(
                    swerveConfig.getSwerveModuleConfigurations()[3],
                    driveSimulation.getModules()[3]) {},
                driveSimulation::setSimulationWorldPose);
        new VisionSubsystem(
            driveSubsystem,
            VisionIOPhotonVisionSim.ofLimelight4(
                VisionConstants.frontCamera,
                VisionConstants.robotToFrontCam,
                driveSimulation::getSimulatedDriveTrainPose),
            VisionIOPhotonVisionSim.ofLimelight3A(
                VisionConstants.frontLeftCamera,
                VisionConstants.robotToFrontLeftCam,
                driveSimulation::getSimulatedDriveTrainPose),
            VisionIOPhotonVisionSim.ofLimelight3A(
                VisionConstants.frontRightCamera,
                VisionConstants.robotToFrontRightCam,
                driveSimulation::getSimulatedDriveTrainPose));
        break;
      default:
        // Replayed robot, disable IO implementations
        driveSubsystem =
            new DriveSubsystem(
                constants,
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                (robotPose) -> {});
        new VisionSubsystem(driveSubsystem);
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(driveSubsystem));
    autoChooser.addOption(
        "Drive Simple FF Characterization",
        DriveCommands.feedforwardCharacterization(driveSubsystem));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        driveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)",
        driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)",
        driveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick} or {@link
   * XboxController}), and then passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    driveSubsystem.setDefaultCommand(
        DriveCommands.joystickDrive(
            driveSubsystem,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                driveSubsystem,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(driveSubsystem::stopWithX, driveSubsystem));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        driveSubsystem.setPose(
                            new Pose2d(
                                driveSubsystem.getPose().getTranslation(), new Rotation2d())),
                    driveSubsystem)
                .ignoringDisable(true));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void resetSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    driveSubsystem.setPose(new Pose2d(3, 3, new Rotation2d()));
    SimulatedArena.getInstance().resetFieldForAuto();
  }

  public void updateSimulation() {
    if (Constants.currentMode != Constants.Mode.SIM) return;

    SimulatedArena.getInstance().simulationPeriodic();
    Logger.recordOutput(
        "FieldSimulation/RobotPosition", driveSimulation.getSimulatedDriveTrainPose());
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
