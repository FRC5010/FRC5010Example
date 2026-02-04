package frc.robot.rebuilt.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.Constants.ClimbConstants;
import frc.robot.rebuilt.subsystems.Climb.Climb;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.arch.StateMachine.State;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.telemetry.DisplayString;
import org.frc5010.common.telemetry.DisplayValuesHelper;

public class ClimbCommands {

  private Map<String, GenericSubsystem> subsystems;
  private StateMachine stateMachine;
  private State idleState;
  private State elevateState;
  private State descendState;
  private State liftedState;
  private State loweredState;
  private DisplayValuesHelper Dashboard;
  private static DisplayString currentState;
  private static DisplayString requestedState;

  private static enum ClimbState {
    IDLE,
    ELEVATE,
    LIFTED,
    DESCEND,
    LOWERED
  }

  private Climb climb;

  public ClimbCommands(Map<String, GenericSubsystem> systems) {
    this.subsystems = systems;

    Dashboard = new DisplayValuesHelper("ClimbCommands");
    currentState = Dashboard.makeDisplayString("Current State");
    currentState.setValue(ClimbState.LOWERED.toString());
    requestedState = Dashboard.makeDisplayString("Requested State");
    requestedState.setValue(ClimbState.LOWERED.toString());

    // Create a simple state machine for climb and set it as the default command for the Climb
    climb = (Climb) subsystems.get(Constants.CLIMB);
    stateMachine = new StateMachine("ClimbStateMachine");
    // a simple idle state; transitions will be added in configureButtonBindings
    idleState =
        stateMachine.addState(
            "idle",
            climb
                .idleCommand()
                .alongWith(Commands.runOnce(() -> setCurrentState(ClimbState.IDLE))));
    loweredState =
        stateMachine.addState(
            "lowered", Commands.runOnce(() -> setCurrentState(ClimbState.LOWERED)));
    liftedState =
        stateMachine.addState("lifted", Commands.runOnce(() -> setCurrentState(ClimbState.LIFTED)));

    // states that actually run the climber
    if (climb != null) {
      elevateState =
          stateMachine.addState(
              "elevate",
              climb
                  .climberCommand(Meters.of(.5))
                  .alongWith(Commands.runOnce(() -> setCurrentState(ClimbState.ELEVATE))));
      descendState =
          stateMachine.addState(
              "lower",
              climb
                  .climberCommand(Meters.of(0))
                  .alongWith(Commands.runOnce(() -> setCurrentState(ClimbState.DESCEND))));
    } else {
      // fallback states if climb isn't available
      elevateState = stateMachine.addState("elevate", Commands.idle());
      descendState = stateMachine.addState("lower", Commands.idle());
    }

    stateMachine.setInitialState(loweredState);

    if (climb != null) {
      stateMachine.addRequirements(climb);
      climb.setDefaultCommand(stateMachine);
    }
  }

  public boolean isRequested(ClimbState state) {
    return ClimbState.valueOf(requestedState.getValue()) == state;
  }

  public boolean isCurrent(ClimbState state) {
    return ClimbState.valueOf(currentState.getValue()) == state;
  }

  private void setCurrentState(ClimbState state) {
    currentState.setValue(state.name());
  }

  private void setRequestedState(ClimbState state) {
    requestedState.setValue(state.name());
  }

  public Command shouldElevateCommand() {
    return Commands.runOnce(() -> setRequestedState(ClimbState.ELEVATE));
  }

  public Command shouldStopCommand() {
    return Commands.runOnce(() -> setRequestedState(ClimbState.IDLE));
  }

  public Command shouldDescendCommand() {
    return Commands.runOnce(() -> setRequestedState(ClimbState.DESCEND));
  }

  public void configureButtonBindings(Controller operator) {
    operator.createXButton().onTrue(shouldElevateCommand()).onFalse(shouldStopCommand());
    operator.createYButton().onTrue(shouldDescendCommand()).onFalse(shouldStopCommand());

    climb.setDefaultCommand(
        Commands.run(
            () -> {
              climb.runClimb(operator.getLeftYAxis());
            },
            climb));

    // lowered -> elevate when requested
    loweredState.switchTo(elevateState).when(() -> isRequested(ClimbState.ELEVATE));
    // descend -> lowered when height is Zero
    descendState.switchTo(loweredState).when(() -> climb.getHeight().isEquivalent(Meters.of(0)));
    // elevate -> Lifted when height is =to Target
    elevateState
        .switchTo(liftedState)
        .when(() -> climb.getHeight().isEquivalent(ClimbConstants.MAX));
    // lifted -> descend when asked to descend
    liftedState.switchTo(descendState).when(() -> isRequested(ClimbState.DESCEND));
    // elevate -> descend when asked to descend
    elevateState.switchTo(descendState).when(() -> isRequested(ClimbState.DESCEND));
    // descend -> elevating when asked to elevate
    descendState.switchTo(elevateState).when(() -> isRequested(ClimbState.ELEVATE));
    // elevate -> idle when stopped
    elevateState.switchTo(idleState).when(() -> isRequested(ClimbState.IDLE));
    // descend -> idle when stopped
    descendState.switchTo(idleState).when(() -> isRequested(ClimbState.IDLE));
    // idle -> elevate when requested
    idleState.switchTo(elevateState).when(() -> isRequested(ClimbState.ELEVATE));
    // idle -> descend when requested
    idleState.switchTo(descendState).when(() -> isRequested(ClimbState.DESCEND));
  }
}
