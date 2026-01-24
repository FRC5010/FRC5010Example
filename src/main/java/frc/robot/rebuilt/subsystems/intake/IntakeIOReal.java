package frc.robot.rebuilt.subsystems.intake;

import java.util.Map;

public class IntakeIOReal implements IntakeIO {
  protected Map<String, Object> devices;

  public IntakeIOReal(Map<String, Object> devices) {
    this.devices = devices;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {}
}
