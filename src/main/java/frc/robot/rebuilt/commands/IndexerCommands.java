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
import org.frc5010.common.telemetry.DisplayValuesHelper;

public class IndexerCommands {
  private Map<String, GenericSubsystem> subsystems;
  private StateMachine stateMachine;
  private DisplayValuesHelper Dashboard;
  private static DisplayString currentState;
  private static DisplayString requestedState;
  private State idleState;
  private State churnState;
  private State feedState;

  private static enum IndexerState {
    IDLE,
    CHURN,
    FEED
  }

  public IndexerCommands(Map<String, GenericSubsystem> systems) {
    this.subsystems = systems;

    Dashboard = new DisplayValuesHelper("IndexerCommands");
    currentState = Dashboard.makeDisplayString("CurrentState");
    currentState.setValue(IndexerState.IDLE.toString());
    requestedState = Dashboard.makeDisplayString("RequestedState");
    requestedState.setValue(IndexerState.IDLE.toString());

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
    idleState.switchTo(churnState).when(() -> isRequested(IndexerState.CHURN));
    idleState.switchTo(feedState).when(() -> isRequested(IndexerState.FEED));
    churnState.switchTo(feedState).when(() -> isRequested(IndexerState.FEED));
    feedState.switchTo(churnState).when(() -> isRequested(IndexerState.CHURN));
  }

  public boolean isRequested(IndexerState state) {
    return IndexerState.valueOf(requestedState.getValue()) == state;
  }

  public boolean isCurrent(IndexerState state) {
    return IndexerState.valueOf(currentState.getValue()) == state;
  }

  private void setCurrentState(IndexerState state) {
    currentState.setValue(state.name());
  }

  private void setRequestedState(IndexerState state) {
    requestedState.setValue(state.name());
  }

  private Command churnStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              setCurrentState(IndexerState.CHURN);
            }));
  }

  private Command idleStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              setCurrentState(IndexerState.IDLE);
            }));
  }

  private Command feedStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              setCurrentState(IndexerState.FEED);
            }));
  }

  public Command shouldIdleCommand() {
    return Commands.runOnce(() -> setRequestedState(IndexerState.IDLE));
  }

  public Command shouldChurnCommand() {
    return Commands.runOnce(() -> setRequestedState(IndexerState.CHURN));
  }

  public Command shouldFeedCommand() {
    return Commands.runOnce(() -> setRequestedState(IndexerState.FEED));
  }
}
