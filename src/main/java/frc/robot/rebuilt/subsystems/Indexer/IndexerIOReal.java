package frc.robot.rebuilt.subsystems.Indexer;

import java.util.Map;
import org.frc5010.common.motors.function.PercentControlMotor;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;

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
    inputs.spindexerSpeed = ((SmartMotorController) Spindexer.getMotorController()).getDutyCycle();
    inputs.transferFrontSpeed = ((SmartMotorController) TransferFront.getMotorController()).getDutyCycle();
    inputs.transferBackSpeed = ((SmartMotorController) TransferBack.getMotorController()).getDutyCycle();

  }

  @Override
  public void runSpindexer(double speed) {
    Spindexer.set(speed);
  }

  @Override
  public void runTransferFront(double speed) {
    ((SmartMotorController) TransferFront.getMotorController()).setDutyCycle(speed);

  }

  @Override
  public void runTransferBack(double speed) {
    TransferBack.set(speed);
  }
}
