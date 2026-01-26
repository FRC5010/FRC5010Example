package frc.robot.rebuilt.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.rebuilt.Constants;
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
  private State lowerState;
  private Boolean requestState;

  public ClimbCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;

    // Create a simple state machine for climb and set it as the default command for the Climb
    Climb climb = (Climb) subsystems.get(Constants.CLIMB);
    stateMachine = new StateMachine("ClimbStateMachine");
    // a simple idle state; transitions will be added in configureButtonBindings
    idleState = stateMachine.addState("idle", Commands.idle());
    // states that actually run the climber
    if (climb != null) {
      elevateState = stateMachine.addState("elevate", climb.climberCommand(.5));
      lowerState = stateMachine.addState("lower", climb.climberCommand(0));
    } else {
      // fallback states if climb isn't available
      elevateState = stateMachine.addState("elevate", Commands.idle());
      lowerState = stateMachine.addState("lower", Commands.idle());
    }

    stateMachine.setInitialState(idleState);

    if (climb != null) {
      stateMachine.addRequirements(climb);
      climb.setDefaultCommand(stateMachine);
    }
  }

  public void configureButtonBindings(Controller controller) {
    // create Trigger objects for clarity and reuse
    Trigger rightBumper = controller.createXButton();
    Trigger leftBumper = controller.createYButton();

    // idle -> elevate on right bumper press
    idleState.switchTo(elevateState).when(rightBumper);
    // elevate -> idle when right bumper released
    elevateState.switchTo(idleState).when(() -> !rightBumper.getAsBoolean());

    // idle -> lower on left bumper press
    idleState.switchTo(lowerState).when(leftBumper);
    // lower -> idle when left bumper released
    lowerState.switchTo(idleState).when(() -> !leftBumper.getAsBoolean());
  }
}
