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

public class IndexerCommands {
  private Map<String, GenericSubsystem> subsystems;
  private StateMachine stateMachine;
  private State idleState;
  private State churnState;
  private State feedState;
  private State forceState;
  private Indexer indexer;

  public static enum IndexerState {
    IDLE,
    CHURN,
    FORCE,
    FEED
  }

  public IndexerCommands(Map<String, GenericSubsystem> systems) {
    this.subsystems = systems;
    this.indexer = (Indexer) subsystems.get(Constants.INDEXER);
    stateMachine = new StateMachine("IndexStateMachine");
    idleState = stateMachine.addState("idle", idleStateCommand());
    if (indexer != null) {
      churnState = stateMachine.addState("churn", churnStateCommand());
      feedState = stateMachine.addState("feed", feedStateCommand());
      forceState = stateMachine.addState("", Commands.idle());
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
    idleState.switchTo(churnState).when(() -> indexer.isRequested(IndexerState.CHURN));
    idleState.switchTo(feedState).when(() -> indexer.isRequested(IndexerState.FEED));
    churnState.switchTo(feedState).when(() -> indexer.isRequested(IndexerState.FEED));
    churnState.switchTo(idleState).when(() -> indexer.isRequested(IndexerState.IDLE));
    feedState.switchTo(idleState).when(() -> indexer.isRequested(IndexerState.IDLE));
    feedState.switchTo(churnState).when(() -> indexer.isRequested(IndexerState.CHURN));
    forceState.switchTo(idleState).when(() -> indexer.isRequested(IndexerState.IDLE));
    driver.createRightBumper().onTrue(shouldChurnCommand()).onFalse(shouldIdleCommand());
  }

  private Command churnStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              indexer.setCurrentState(IndexerState.CHURN);
            }));
  }

  private Command idleStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              indexer.setCurrentState(IndexerState.IDLE);
            }));
  }

  private Command feedStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              indexer.setCurrentState(IndexerState.FEED);
            }));
  }

  public Command shouldIdleCommand() {
    return Commands.runOnce(() -> indexer.setRequestedState(IndexerState.IDLE));
  }

  public Command shouldChurnCommand() {
    return Commands.runOnce(() -> indexer.setRequestedState(IndexerState.CHURN));
  }

  public Command shouldFeedCommand() {
    return Commands.runOnce(() -> indexer.setRequestedState(IndexerState.FEED));
  }
}
