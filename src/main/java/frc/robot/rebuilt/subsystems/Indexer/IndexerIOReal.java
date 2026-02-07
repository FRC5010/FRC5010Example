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

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.spindexerSpeed = Spindexer.get();
    inputs.transferFrontSpeed = TransferFront.get();
    inputs.transferBackSpeed = TransferBack.get();

  }

  @Override
  public void runSpindexer(double speed) {
    Spindexer.set(speed);
  }

  @Override
  public void runTransferFront(double speed) {
    TransferFront.set(speed);

  }

  @Override
  public void runTransferBack(double speed) {
    TransferBack.set(speed);
  }
}
