package frc.robot.rebuilt.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.Constants.ClimbConstants;
import frc.robot.rebuilt.subsystems.Climb.Climb;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.arch.StateMachine.State;
import org.frc5010.common.sensors.Controller;

public class ClimbCommands {

  private Map<String, GenericSubsystem> subsystems;
  private StateMachine stateMachine;
  private State idleState;
  private State elevateState;
  private State descendState;
  private State liftedState;
  private State loweredState;

  private static enum ClimbState {
    IDLE,
    ELEVATE,
    LIFTED,
    DESCEND,
    LOWERED
  }

  Climb climb;

  private static ClimbState requestedState = ClimbState.LOWERED;

  public ClimbCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;

    // Create a simple state machine for climb and set it as the default command for the Climb
    climb = (Climb) subsystems.get(Constants.CLIMB);
    stateMachine = new StateMachine("ClimbStateMachine");
    // a simple idle state; transitions will be added in configureButtonBindings
    loweredState = stateMachine.addState("lowered", Commands.idle());
    // states that actually run the climber
    if (climb != null) {
      elevateState = stateMachine.addState("elevate", climb.climberCommand(.5));
      descendState = stateMachine.addState("lower", climb.climberCommand(0));
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

  public Command shouldElevateCommand() {
    return Commands.runOnce(() -> requestedState = ClimbState.ELEVATE);
  }

  public Command shouldStopCommand() {
    return Commands.runOnce(() -> requestedState = ClimbState.IDLE);
  }

  public Command shouldDescendCommand() {
    return Commands.runOnce(() -> requestedState = ClimbState.DESCEND);
  }

  public void configureButtonBindings(Controller operator) {
    operator.createXButton().onTrue(shouldElevateCommand()).onFalse(shouldStopCommand());

    operator.createYButton().onTrue(shouldDescendCommand()).onFalse(shouldStopCommand());

    // create Trigger objects for clarity and reuse
    Trigger rightBumper = operator.createXButton();
    Trigger leftBumper = operator.createYButton();

    // lowered -> elevate when requested
    loweredState.switchTo(elevateState).when(() -> requestedState == ClimbState.ELEVATE);
    // descend -> lowered when height is Zero
    descendState.switchTo(loweredState).when(() -> climb.getHeight().isEquivalent(Meters.of(0)));
    // elevate -> Lifted when height is =to Target
    elevateState.switchTo(liftedState).when(() -> climb.getHeight().isEquivalent(ClimbConstants.MAX));
    // lifted -> descend when asked to descend
    liftedState.switchTo(descendState).when(() -> requestedState == ClimbState.DESCEND);
    // elevate -> descend when asked to descend
    elevateState.switchTo(descendState).when(() -> requestedState == ClimbState.DESCEND);
    // descend -> elevating when asked to elevate
    descendState.switchTo(elevateState).when(() -> requestedState == ClimbState.ELEVATE);
    // elevate -> idle when stopped
    elevateState.switchTo(idleState).when(() -> requestedState == ClimbState.IDLE);
    // descend -> idle when stopped
    descendState.switchTo(idleState).when(() -> requestedState == ClimbState.IDLE);
  }
}
