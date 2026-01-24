package frc.robot.rebuilt.commands;

import java.util.Map;

public class IndexerIOReal implements IndexerIO {
  protected Map<String, Object> devices;

  public IndexerIOReal(Map<String, Object> devices) {
    this.devices = devices;
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {}
}
