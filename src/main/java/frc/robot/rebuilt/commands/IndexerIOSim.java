package frc.robot.rebuilt.commands;

import java.util.Map;

public class IndexerIOSim extends IndexerIOReal {
  protected Map<String, Object> devices;

  public IndexerIOSim(Map<String, Object> devices) {
    super(devices);
  }
}
