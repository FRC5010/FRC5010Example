package frc.robot.rebuilt.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {}

  public default void updateInputs(IndexerIOInputs inputs) {}
  ;
}
