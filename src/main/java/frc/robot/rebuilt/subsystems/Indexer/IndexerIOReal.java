package frc.robot.rebuilt.subsystems.indexer;

import java.util.Map;
import org.frc5010.common.motors.function.PercentControlMotor;
/** Implements the hardware Indexer IO */
public class IndexerIOReal implements IndexerIO {
  protected Map<String, Object> devices;
  private PercentControlMotor Spindexer;
  private PercentControlMotor TransferFront, TransferBack;

  public IndexerIOReal(Map<String, Object> devices) {
    Spindexer = (PercentControlMotor) devices.get("spindexer");
    TransferFront = (PercentControlMotor) devices.get("transfer_front");
    TransferBack = (PercentControlMotor) devices.get("transfer_back");
    TransferFront.setFollow(TransferBack, true);
    this.devices = devices;
  }
/** Updates indexer input values with current motor speed*/
  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.spindexerSpeed = Spindexer.get();
    inputs.transferFrontSpeed = TransferFront.get();
    inputs.transferBackSpeed = TransferBack.get();
  }
/** Sets the spindexer motor speed */
  @Override
  public void runSpindexer(double speed) {
    Spindexer.set(speed);
  }
/** Sets the front transfer motor speed*/
  @Override
  public void runTransferFront(double speed) {
    TransferFront.set(speed);
  }
/** Sets the back transfer motor speed */
  @Override
  public void runTransferBack(double speed) {
    TransferBack.set(speed);
  }
}
