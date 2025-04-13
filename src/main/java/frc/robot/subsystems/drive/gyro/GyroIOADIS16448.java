package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import frc.robot.subsystems.drive.PhoenixOdometryThread;
import java.util.Queue;

/**
 * IO implementation for <a href="https://www.analog.com/en/products/adis16448.html">ADIS16448</a>.
 */
public class GyroIOADIS16448 implements GyroIO {
  private final ADIS16448_IMU gyro = new ADIS16448_IMU();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIOADIS16448() {
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(gyro::getAngle);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees(gyro.getAngle());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(gyro.getRate());
    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
