package frc.robot.rebuilt.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.arch.StateMachine.State;
import org.frc5010.common.sensors.Controller;
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
    idleState = stateMachine.addState("idle", idleStateCommand());
    if (indexer != null) {
      churnState = stateMachine.addState("churn", churnStateCommand());
      feedState = stateMachine.addState("feed", feedStateCommand());
    }
    stateMachine.setInitialState(idleState);

    if (indexer != null) {
      stateMachine.addRequirements(indexer);
      indexer.setDefaultCommand(stateMachine);
    }
  }
  // TODO: Adjust Button Inputs
  public void configureButtonBindings(Controller driver, Controller operator) {
    driver.createBButton().onTrue(shouldChurnCommand()).onFalse(shouldIdleCommand());
    idleState.switchTo(churnState).when(() -> requestedState == IndexerState.CHURN);
    idleState.switchTo(feedState).when(() -> requestedState == IndexerState.FEED);
    churnState.switchTo(feedState).when(() -> requestedState == IndexerState.FEED);
    feedState.switchTo(churnState).when(() -> requestedState == IndexerState.CHURN);
  }

  private Command churnStateCommand() {
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

  private Command feedStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              commandState.setValue("Feed");
              currentState = IndexerState.FEED;
            }));
  }

  public Command shouldIdleCommand() {
    return Commands.runOnce(() -> requestedState = IndexerState.IDLE);
  }

  public Command shouldChurnCommand() {
    return Commands.runOnce(() -> requestedState = IndexerState.CHURN);
  }

  public Command shouldFeedCommand() {
    return Commands.runOnce(() -> requestedState = IndexerState.FEED);
  }
}
