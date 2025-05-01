package frc.robot.config;

import edu.wpi.first.units.measure.Frequency;
import frc.robot.subsystems.drive.gyro.*;
import frc.robot.util.drivers.CanDeviceId;
import java.util.function.Function;
import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroConfiguration {
  public enum GyroType {
    PIGEON_2(GyroIOPigeon2::new, COTS.ofPigeon2()),
    NAV_X(GyroIONavX::new, COTS.ofNav2X());

    private final Function<GyroConfiguration, GyroIO> gyroSupplier;
    private final Supplier<GyroSimulation> gyroSimulation;

    GyroType(
        Function<GyroConfiguration, GyroIO> gyroSupplier, Supplier<GyroSimulation> gyroSimulation) {
      this.gyroSupplier = gyroSupplier;
      this.gyroSimulation = gyroSimulation;
    }

    public GyroIO createIO(GyroConfiguration constants) {
      return gyroSupplier.apply(constants);
    }

    public GyroSimulation getGyroSimulation() {
      return gyroSimulation.get();
    }
  }

  private final GyroType gyroType;
  private final Frequency pollingRate;
  private final CanDeviceId gyroCanDeviceId;

  private GyroConfiguration(Builder builder) {
    this.gyroType = builder.gyroType;
    this.pollingRate = builder.pollingRate;
    this.gyroCanDeviceId = builder.gyroCanDeviceId;
  }

  public static class Builder {
    private GyroType gyroType;
    private Frequency pollingRate;
    private CanDeviceId gyroCanDeviceId;

    public Builder setGyroType(GyroType gyroType) {
      this.gyroType = gyroType;
      return this;
    }

    public Builder setPollingRate(Frequency pollingRate) {
      this.pollingRate = pollingRate;
      return this;
    }

    public Builder setGyroCanDeviceId(CanDeviceId canDeviceId) {
      this.gyroCanDeviceId = canDeviceId;
      return this;
    }

    public GyroConfiguration build() {
      return new GyroConfiguration(this);
    }
  }

  public GyroType getGyroType() {
    return gyroType;
  }

  public Frequency getPollingRate() {
    return pollingRate;
  }

  public CanDeviceId getGyroCanDeviceId() {
    return gyroCanDeviceId;
  }
}
