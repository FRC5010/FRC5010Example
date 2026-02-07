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
  private static Indexer indexer;

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
    driver.createRightBumper().onTrue(shouldForceCommand()).onFalse(shouldChurnCommand());
    operator.createRightBumper().onTrue(shouldForceCommand()).onFalse(shouldChurnCommand());
    // driver.createRightBumper().onTrue(shouldChurnCommand()).onFalse(shouldIdleCommand());
    // conflicting actions one changes to Force other Churn
    idleState.switchTo(churnState).when(() -> indexer.isRequested(IndexerState.CHURN));
    idleState.switchTo(feedState).when(() -> indexer.isRequested(IndexerState.FEED));
    idleState.switchTo(forceState).when(() -> indexer.isRequested(IndexerState.FORCE));
    churnState.switchTo(feedState).when(() -> indexer.isRequested(IndexerState.FEED));
    churnState.switchTo(idleState).when(() -> indexer.isRequested(IndexerState.IDLE));
    churnState.switchTo(forceState).when(() -> indexer.isRequested(IndexerState.FORCE));
    feedState.switchTo(idleState).when(() -> indexer.isRequested(IndexerState.IDLE));
    feedState.switchTo(churnState).when(() -> indexer.isRequested(IndexerState.CHURN));
    feedState.switchTo(forceState).when(() -> indexer.isRequested(IndexerState.FORCE));
    forceState.switchTo(idleState).when(() -> indexer.isRequested(IndexerState.IDLE));
    forceState.switchTo(churnState).when(() -> indexer.isRequested(IndexerState.CHURN));
  }

  public static Command forceStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              indexer.setCurrentState(IndexerState.FORCE);
              indexer.runSpindexer(0.50);
              indexer.runTransferFront(0.50);
              indexer.runTransferBack(0.50);
            }));
  }

  private static Command churnStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              indexer.setCurrentState(IndexerState.CHURN);
              indexer.runSpindexer(0);
              indexer.runTransferFront(0.25);
              indexer.runTransferFront(0.25);
            }));
  }

  private static Command idleStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              indexer.setCurrentState(IndexerState.IDLE);
              indexer.runSpindexer(0);
              indexer.runTransferFront(0);
              indexer.runTransferFront(0);
            }));
  }

  private static Command feedStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              indexer.setCurrentState(IndexerState.FEED);
              indexer.runSpindexer(0.5);
              indexer.runTransferFront(0.5);
              indexer.runTransferFront(0.5);
            }));
  }

  public static Command shouldIdleCommand() {
    return Commands.runOnce(() -> indexer.setRequestedState(IndexerState.IDLE));
  }

  public static Command shouldChurnCommand() {
    return Commands.runOnce(() -> indexer.setRequestedState(IndexerState.CHURN));
  }

  public static Command shouldFeedCommand() {
    return Commands.runOnce(() -> indexer.setRequestedState(IndexerState.FEED));
  }

  public static Command shouldForceCommand() {
    return Commands.runOnce(() -> indexer.setRequestedState(IndexerState.FORCE));
  }
}
