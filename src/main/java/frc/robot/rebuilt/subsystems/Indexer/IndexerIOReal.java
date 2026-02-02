package frc.robot.rebuilt.subsystems.Indexer;

import java.util.Map;
import org.frc5010.common.motors.function.PercentControlMotor;

public class IndexerIOReal implements IndexerIO {
  protected Map<String, Object> devices;
  private PercentControlMotor Spindexer;
  private PercentControlMotor Feeder;

  public IndexerIOReal(Map<String, Object> devices) {
    this.devices = devices;
  }

  public void RunFeeder(double speed) {
    Spindexer.set(speed);
  }

  public void RunSpindexer(double speed) {
    Feeder.set(speed);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // TODO: Add update to input values
  }
}
