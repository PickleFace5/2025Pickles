package frc.robot.subsystems.drive.gyro;

import java.util.function.Supplier;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public enum GyroType {
  PIGEON_2(GyroIOPigeon2::new, COTS.ofPigeon2()),
  NAVX2(GyroIONavX::new, COTS.ofNav2X()),
  ADIS16448(GyroIOADIS16448::new, COTS.ofGenericGyro()),
  ADIS16470(GyroIOADIS16470::new, COTS.ofGenericGyro());

  private final Supplier<GyroIO> gyroSupplier;
  private final Supplier<GyroSimulation> gyroSimulation;

  GyroType(Supplier<GyroIO> gyroSupplier, Supplier<GyroSimulation> gyroSimulation) {
    this.gyroSupplier = gyroSupplier;
    this.gyroSimulation = gyroSimulation;
  }

  public GyroIO createIO() {
    return gyroSupplier.get();
  }

  public GyroSimulation getGyroSimulation() {
    return gyroSimulation.get();
  }
}
