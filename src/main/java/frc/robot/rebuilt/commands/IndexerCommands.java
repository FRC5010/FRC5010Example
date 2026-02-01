package frc.robot.rebuilt.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.arch.StateMachine.State;

public class IndexerCommands {
  private Map<String, GenericSubsystem> subsystems;
  private StateMachine stateMachine;
  private State idleState;
  private State churnState;
  private State feedState;

  public IndexerCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;

    // Create a simple state machine for the indexer and set it as the default command.
    Indexer indexer = (Indexer) subsystems.get(Constants.INDEXER);
    stateMachine = new StateMachine("IndexStateMachine");

    idleState = stateMachine.addState("idle", Commands.idle());
    if (indexer != null) {
      churnState = stateMachine.addState("churn", indexer.spindexerCommand(0));
      feedState = stateMachine.addState("feed", indexer.spindexerCommand(1));
    }
    stateMachine.setInitialState(idleState);

    if (indexer != null) {
      stateMachine.addRequirements(indexer);
      indexer.setDefaultCommand(stateMachine);
    }
  }
}
