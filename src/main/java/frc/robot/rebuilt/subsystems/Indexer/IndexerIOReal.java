package frc.robot.rebuilt.subsystems.Indexer;

import java.util.Map;

import frc.robot.rebuilt.subsystems.Indexer.IndexerIO.IndexerIOInputs;

public class IndexerIOReal implements IndexerIO {
  protected Map<String, Object> devices;

  public IndexerIOReal(Map<String, Object> devices) {
    this.devices = devices;
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {}
}
