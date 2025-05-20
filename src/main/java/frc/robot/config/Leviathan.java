package frc.robot.config;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import frc.robot.util.drivers.CanDeviceId;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;

public class Leviathan implements RobotConstants {
  private static final String CANIVORE_NAME = "Drivetrain";

  private static final GyroConfiguration.GyroType GYRO_TYPE = GyroConfiguration.GyroType.PIGEON_2;

  // Ports and IDs
  private static final CanDeviceId GYRO = new CanDeviceId(9, CANIVORE_NAME);

  private static final CanDeviceId FRONT_LEFT_DRIVE_MOTOR = new CanDeviceId(3, CANIVORE_NAME);
  private static final CanDeviceId FRONT_LEFT_STEER_MOTOR = new CanDeviceId(7, CANIVORE_NAME);
  private static final CanDeviceId FRONT_LEFT_STEER_ENCODER = new CanDeviceId(7, CANIVORE_NAME);

  private static final CanDeviceId FRONT_RIGHT_DRIVE_MOTOR = new CanDeviceId(1, CANIVORE_NAME);
  private static final CanDeviceId FRONT_RIGHT_STEER_MOTOR = new CanDeviceId(5, CANIVORE_NAME);
  private static final CanDeviceId FRONT_RIGHT_STEER_ENCODER = new CanDeviceId(5, CANIVORE_NAME);

  private static final CanDeviceId BACK_LEFT_DRIVE_MOTOR = new CanDeviceId(4, CANIVORE_NAME);
  private static final CanDeviceId BACK_LEFT_STEER_MOTOR = new CanDeviceId(8, CANIVORE_NAME);
  private static final CanDeviceId BACK_LEFT_STEER_ENCODER = new CanDeviceId(8, CANIVORE_NAME);

  private static final CanDeviceId BACK_RIGHT_DRIVE_MOTOR = new CanDeviceId(2, CANIVORE_NAME);
  private static final CanDeviceId BACK_RIGHT_STEER_MOTOR = new CanDeviceId(6, CANIVORE_NAME);
  private static final CanDeviceId BACK_RIGHT_STEER_ENCODER = new CanDeviceId(6, CANIVORE_NAME);

  // ==================================================================================
  // Physical Robot Constants
  // ==================================================================================

  private static final Mass ROBOT_MASS = Kilograms.of(74.088);
  private static final MomentOfInertia ROBOT_MOI = KilogramSquareMeters.of(6.883);
  private static final Distance BUMPER_LENGTH = Inches.of(37);

  // ==================================================================================
  // Physical Swerve Module constants
  // ==================================================================================

  /**
   * Wheel radius in meters. Accuracy in these measurements affects wheel odometry which measures
   * distance as a function of the number of rotations * wheel circumference.
   */
  private static final Distance WHEEL_RADIUS = Inches.of(2);

  /** Ratio between the drive motor shaft and the output shaft the wheel is mounted on. */
  private static final double DRIVE_GEAR_RATIO = 6.746031746031747;

  /** Ratio between the steer motor shaft and the steer output shaft. */
  private static final double STEER_GEAR_RATIO = 150.0 / 7.0;

  /**
   * The coupled gear ratio between the CanCoder and the drive motor. Every 1 rotation of the steer
   * motor results in coupled ratio of drive turns.
   */
  private static final double COUPLING_GEAR_RATIO = 3.5714285714285716;

  // ==================================================================================
  // Physical Drivetrain constants
  // ==================================================================================

  /**
   * Wheelbase length is the distance between the front and back wheels. Positive x values represent
   * moving towards the front of the robot
   */
  private static final Distance WHEELBASE_LENGTH_METERS = Inches.of(24);

  /**
   * Wheel track width is the distance between the left and right wheels. Positive y values
   * represent moving towards the left of the robot.
   */
  private static final Distance WHEEL_TRACK_WIDTH_METERS = Inches.of(24);

  private static final Translation2d[] MODULE_LOCATIONS =
      new Translation2d[] {
        new Translation2d(WHEELBASE_LENGTH_METERS.div(2), WHEELBASE_LENGTH_METERS.div(2)),
        new Translation2d(
            WHEELBASE_LENGTH_METERS.div(2), WHEELBASE_LENGTH_METERS.div(2).unaryMinus()),
        new Translation2d(
            WHEELBASE_LENGTH_METERS.div(2).unaryMinus(), WHEELBASE_LENGTH_METERS.div(2)),
        new Translation2d(
            WHEELBASE_LENGTH_METERS.div(2).unaryMinus(),
            WHEELBASE_LENGTH_METERS.div(2).unaryMinus())
      };

  /** The maximum linear speed of the robot in meters per second. */
  private static final LinearVelocity MAX_SPEED = MetersPerSecond.of(3.5);

  /** The maximum linear acceleration of the robot in meters per second squared. */
  private static final LinearAcceleration MAX_ACCEL = MetersPerSecondPerSecond.of(7.0);

  // CANcoder offsets of the swerve modules
  private static final Angle FRONT_LEFT_ENCODER_OFFSET = Rotations.of(0.155029296875);
  private static final Angle FRONT_RIGHT_ENCODER_OFFSET = Rotations.of(-0.27978515625);
  private static final Angle BACK_LEFT_ENCODER_OFFSET = Rotations.of(-0.241943359375);
  private static final Angle BACK_RIGHT_ENCODER_OFFSET = Rotations.of(-0.06494140625);

  private static final SwerveModuleConfiguration MASTER_MODULE_CONFIG =
      new SwerveModuleConfiguration.Builder()
          .setDriveGearRatio(DRIVE_GEAR_RATIO)
          .setSteerGearRatio(STEER_GEAR_RATIO)
          .setCouplingGearRatio(COUPLING_GEAR_RATIO)
          .setWheelRadius(WHEEL_RADIUS)
          .setDriveMotorGains(new ConfigureSlot0Gains(0.2, 0.0, 0.0, 0.141, 0.82, 0.0))
          .setSteerMotorGains(new ConfigureSlot0Gains(100, 0.0, 0.3, 0.14, 1.91, 0.0))
          .setDriveMotorInverted(true)
          .setSteerMotorStatorCurrent(Amps.of(60.0))
          .setSteerMotorInverted(true)
          .setSpeedAt12Volts(MAX_SPEED)
          .setFeedbackSource(SwerveModuleConfiguration.SteerFeedbackType.FusedCANcoder)
          .build();

  // MapleSim Configuration
  private static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withGyro(GYRO_TYPE::getGyroSimulation)
          .withSwerveModule(
              COTS.ofMark4i(
                  DCMotor.getKrakenX60Foc(1),
                  DCMotor.getKrakenX60Foc(1),
                  COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
                  2))
          .withTrackLengthTrackWidth(WHEELBASE_LENGTH_METERS, WHEEL_TRACK_WIDTH_METERS)
          .withBumperSize(BUMPER_LENGTH, BUMPER_LENGTH)
          .withRobotMass(ROBOT_MASS);

  // PathPlanner Config
  private static final RobotConfig pathplannerConfig =
      new RobotConfig(
          ROBOT_MASS,
          ROBOT_MOI,
          new ModuleConfig(
              WHEEL_RADIUS,
              MAX_SPEED,
              COTS.WHEELS.DEFAULT_NEOPRENE_TREAD.cof,
              DCMotor.getKrakenX60Foc(1).withReduction(DRIVE_GEAR_RATIO),
              Amps.of(80),
              1),
          MODULE_LOCATIONS);

  // Robot configuration
  private final DrivetrainConfiguration drivetrainConfiguration =
      new DrivetrainConfiguration.Builder(CANIVORE_NAME)
          // In order of front left, front right, back left, back right
          .setModuleLocations(MODULE_LOCATIONS)
          .setDiscretizeChassisSpeeds(true)
          .setMaxChassisSpeed(MAX_SPEED)
          .setMaxChassisAccel(MAX_ACCEL)
          .setMaxChassisAngularSpeed(
              RadiansPerSecond.of(MAX_SPEED.in(MetersPerSecond) / WHEEL_RADIUS.in(Meters)))
          .setMaxChassisAngularAccel(
              RadiansPerSecondPerSecond.of(
                  Math.pow(MAX_SPEED.in(MetersPerSecond), 2) / WHEEL_RADIUS.in(Meters)))
          .setKinematics(
              new SwerveDriveKinematics(
                  new Translation2d(
                      WHEELBASE_LENGTH_METERS.div(2), WHEEL_TRACK_WIDTH_METERS.div(2)),
                  new Translation2d(
                      WHEELBASE_LENGTH_METERS.div(2), WHEEL_TRACK_WIDTH_METERS.div(2).unaryMinus()),
                  new Translation2d(
                      WHEEL_TRACK_WIDTH_METERS.div(2).unaryMinus(),
                      WHEEL_TRACK_WIDTH_METERS.div(2)),
                  new Translation2d(
                      WHEEL_TRACK_WIDTH_METERS.div(2).unaryMinus(),
                      WHEEL_TRACK_WIDTH_METERS.div(2).unaryMinus())))
          .setGyroConfiguration(
              new GyroConfiguration.Builder(GYRO_TYPE)
                  .setPollingRate(Hertz.of(250))
                  .setGyroCanDeviceId(GYRO)
                  .build())
          // In order of front left, front right, back left, back right
          .setSwerveModuleConfigurations(
              new SwerveModuleConfiguration[] {
                new SwerveModuleConfiguration.Builder(MASTER_MODULE_CONFIG)
                    .setName("Front Left")
                    .setDriveCanDeviceId(FRONT_LEFT_DRIVE_MOTOR)
                    .setSteerCanDeviceId(FRONT_LEFT_STEER_MOTOR)
                    .setEncoderCanDeviceId(FRONT_LEFT_STEER_ENCODER)
                    .setEncoderOffset(FRONT_LEFT_ENCODER_OFFSET)
                    .build(),
                new SwerveModuleConfiguration.Builder(MASTER_MODULE_CONFIG)
                    .setName("Front Right")
                    .setDriveCanDeviceId(FRONT_RIGHT_DRIVE_MOTOR)
                    .setSteerCanDeviceId(FRONT_RIGHT_STEER_MOTOR)
                    .setEncoderCanDeviceId(FRONT_RIGHT_STEER_ENCODER)
                    .setEncoderOffset(FRONT_RIGHT_ENCODER_OFFSET)
                    .build(),
                new SwerveModuleConfiguration.Builder(MASTER_MODULE_CONFIG)
                    .setName("Back Left")
                    .setDriveCanDeviceId(BACK_LEFT_DRIVE_MOTOR)
                    .setSteerCanDeviceId(BACK_LEFT_STEER_MOTOR)
                    .setEncoderCanDeviceId(BACK_LEFT_STEER_ENCODER)
                    .setEncoderOffset(BACK_LEFT_ENCODER_OFFSET)
                    .build(),
                new SwerveModuleConfiguration.Builder(MASTER_MODULE_CONFIG)
                    .setName("Back Right")
                    .setDriveCanDeviceId(BACK_RIGHT_DRIVE_MOTOR)
                    .setSteerCanDeviceId(BACK_RIGHT_STEER_MOTOR)
                    .setEncoderCanDeviceId(BACK_RIGHT_STEER_ENCODER)
                    .setEncoderOffset(BACK_RIGHT_ENCODER_OFFSET)
                    .build()
              })
          .build();

  @Override
  public DrivetrainConfiguration getDrivetrainConfiguration() {
    return drivetrainConfiguration;
  }

  @Override
  public DriveTrainSimulationConfig getMapleSimConfig() {
    return mapleSimConfig;
  }

  @Override
  public RobotConfig getPathPlannerConfig() {
    return pathplannerConfig;
  }
}
