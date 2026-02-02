package frc.robot.rebuilt.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.arch.StateMachine.State;
import org.frc5010.common.telemetry.DisplayString;
import org.littletonrobotics.junction.AutoLogOutput;

public class IndexerCommands {
  private Map<String, GenericSubsystem> subsystems;
  private StateMachine stateMachine;
  private DisplayString commandState;
  private State idleState;
  private State churnState;
  private State feedState;
  public static IndexerState currentState = IndexerState.IDLE;

  private static enum IndexerState {
    IDLE,
    CHURN,
    FEED
  }

  @AutoLogOutput(key = "IndexerCommands/RequestedIndexerState")
  private static IndexerState requestedState = IndexerState.IDLE;

  public IndexerCommands(Map<String, GenericSubsystem> systems) {
    this.subsystems = systems;

    // Create a simple state machine for the indexer and set it as the default command.
    Indexer indexer = (Indexer) subsystems.get(Constants.INDEXER);
    stateMachine = new StateMachine("IndexStateMachine");

    idleState = stateMachine.addState("idle", Commands.idle());
    if (indexer != null) {
      churnState = stateMachine.addState("churn", indexer.spindexerCommand(.2));
      feedState = stateMachine.addState("feed", indexer.spindexerCommand(.2));
    }
    stateMachine.setInitialState(idleState);

    if (indexer != null) {
      stateMachine.addRequirements(indexer);
      indexer.setDefaultCommand(stateMachine);
    }
  }

  private Command churnStatecommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              commandState.setValue("Churn");
              currentState = IndexerState.CHURN;
            }));
  }

  private Command idleStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              commandState.setValue("Idle");
              currentState = IndexerState.IDLE;
            }));
  }
}
