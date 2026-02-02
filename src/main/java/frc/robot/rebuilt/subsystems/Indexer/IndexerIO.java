package frc.robot.rebuilt.subsystems.Indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  public static class IndexerIOInputs {
    public double spindexerSpeed = 0;
    public double transferFrontSpeed = 0;
    public double transferBackSpeed = 0;
  }

  public void RunSpindexer(double speed);

  public void RunTransferFront(double speed);

  public void RunTransferBack(double speed);

  public default void updateInputs(IndexerIOInputs inputs) {}
}
