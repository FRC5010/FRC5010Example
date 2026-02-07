package frc.robot.rebuilt.subsystems.Indexer;

import java.util.Map;
import org.frc5010.common.motors.function.PercentControlMotor;

public class IndexerIOReal implements IndexerIO {
  protected Map<String, Object> devices;
  private PercentControlMotor Spindexer;
  private PercentControlMotor TransferFront, TransferBack;

  public IndexerIOReal(Map<String, Object> devices) {
    Spindexer = (PercentControlMotor) devices.get("spindexer");
    TransferFront = (PercentControlMotor) devices.get("transfer_front");
    TransferBack = (PercentControlMotor) devices.get("transfer_back");
    this.devices = devices;
  }

  public void RunTransferFront(double speed) {
    TransferFront.set(speed);
  }

  public void RunTransferBack(double speed) {
    TransferBack.set(speed);
  }

  public void RunSpindexer(double speed) {
    Spindexer.set(speed);
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    // TODO: Add update to input values
  }

  @Override
  public void runSpindexer(double speed) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runSpindexer'");
  }

  @Override
  public void runTransferFront(double speed) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runTransferFront'");
  }

  @Override
  public void runTransferBack(double speed) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'runTransferBack'");
  }
}
