package frc.robot.rebuilt.commands;

import java.util.Map;

import org.frc5010.common.arch.GenericSubsystem;

public class IndexerCommands {
  private Map<String, GenericSubsystem> subsystems;

  public IndexerCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;
  }

}
