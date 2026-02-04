package frc.robot.rebuilt.subsystems.Indexer;

import frc.robot.rebuilt.commands.IndexerCommands;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double spindexerSpeed = 0;
    public double transferFrontSpeed = 0;
    public double transferBackSpeed = 0;
    public IndexerCommands.IndexerState stateRequested = IndexerCommands.IndexerState.IDLE;
    public IndexerCommands.IndexerState stateCurrent = IndexerCommands.IndexerState.IDLE;
  }

  public void RunSpindexer(double speed);

  public void RunTransferFront(double speed);

  public void RunTransferBack(double speed);

  public default void updateInputs(IndexerIOInputs inputs) {}
}
