package frc.robot.rebuilt.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.subsystems.intake.Intake;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.arch.StateMachine.State;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.telemetry.DisplayString;
import org.frc5010.common.telemetry.DisplayValuesHelper;

public class IntakeCommands {
  private DisplayValuesHelper Dashboard;
  Intake intake;
  Map<String, GenericSubsystem> subsystems;
  StateMachine intakeStateMachine = new StateMachine("IntakeStateMachine");
  State retracted = intakeStateMachine.addState("retracted", retractedCommand());
  State outtaking = intakeStateMachine.addState("outtaking", outtakingCommand());
  State retracting = intakeStateMachine.addState("retracting", retractingCommand());
  State intaking = intakeStateMachine.addState("intaking", intakingCommand());

  private static enum IntakeState {
    RETRACTED,
    RETRACTING,
    INTAKING,
    OUTTAKING;
  }

  private DisplayString currentState;
  private DisplayString requestedState;

  public IntakeCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;
    Dashboard = new DisplayValuesHelper("IntakeCommands");
    currentState = Dashboard.makeDisplayString("CurrentState");
    setCurrentState(IntakeState.RETRACTED);
    requestedState = Dashboard.makeDisplayString("RequestedState");
    setRequestedState(IntakeState.RETRACTED);

    intake = (Intake) subsystems.get(Constants.INTAKE);
    intakeStateMachine.setInitialState(retracted);
    if (intake != null) {
      intakeStateMachine.addRequirements(intake);
      intake.setDefaultCommand(intakeStateMachine);
    }
  }

  public void configureButtonBindings(Controller controller) {
    controller.setRightTrigger(controller.createRightTrigger());
    Trigger rightTrigger = new Trigger(() -> controller.getRightTrigger() > 0.25);
    controller.setLeftTrigger(controller.createLeftTrigger());
    Trigger leftTrigger = new Trigger(() -> controller.getLeftTrigger() > 0.25);

    rightTrigger.onTrue(shouldIntaking());
    leftTrigger.onTrue(shouldOuttaking());
    retracting.switchTo(intaking).when(() -> isRequested(IntakeState.INTAKING));
    retracted.switchTo(intaking).when(() -> isRequested(IntakeState.INTAKING));
    intaking.switchTo(retracting).when(() -> isRequested(IntakeState.RETRACTING));
    intaking.switchTo(outtaking).when(() -> isRequested(IntakeState.OUTTAKING));
    outtaking.switchTo(retracting).when(() -> isRequested(IntakeState.RETRACTING));
    outtaking.switchTo(intaking).when(() -> isRequested(IntakeState.INTAKING));
    retracting.switchTo(retracted).when(() -> intake.isRetracted());
  }

  public boolean isRequested(IntakeState state) {
    return IntakeState.valueOf(requestedState.getValue()) == state;
  }

  public boolean isCurrent(IntakeState state) {
    return IntakeState.valueOf(currentState.getValue()) == state;
  }

  private void setCurrentState(IntakeState state) {
    currentState.setValue(state.name());
  }

  private void setRequestedState(IntakeState state) {
    requestedState.setValue(state.name());
  }

  public Command outtakingCommand() {
    return Commands.runOnce(() -> setCurrentState(IntakeState.OUTTAKING))
        .andThen(() -> intake.setPinionPosition(12))
        .andThen(() -> intake.runSpintake(-0.25));
    // assuming outtaking is just intaking but goes the other way
  }

  public Command intakingCommand() {
    return Commands.runOnce(() -> setCurrentState(IntakeState.INTAKING))
        .andThen(() -> intake.setPinionPosition(12))
        .andThen(() -> intake.runSpintake(0.25));
  }

  public Command retractingCommand() {
    return Commands.runOnce(() -> setCurrentState(IntakeState.RETRACTING))
        .andThen(() -> intake.setPinionPosition(0))
        .andThen(() -> intake.runSpintake(0));
  }

  public Command retractedCommand() {
    return Commands.runOnce(() -> setCurrentState(IntakeState.RETRACTED))
        .andThen(() -> intake.setPinionPosition(0))
        .andThen(() -> intake.runSpintake(0));
  }

  public Command shouldOuttaking() {
    return Commands.runOnce(() -> setRequestedState(IntakeState.OUTTAKING));
  }

  public Command shouldIntaking() {
    return Commands.runOnce(() -> setRequestedState(IntakeState.INTAKING));
  }

  public Command shouldRetracting() {
    return Commands.runOnce(() -> setRequestedState(IntakeState.RETRACTING));
  }

  public Command shouldRetracted() {
    return Commands.runOnce(() -> setRequestedState(IntakeState.RETRACTED));
  }
}
