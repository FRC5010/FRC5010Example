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
import org.littletonrobotics.junction.AutoLogOutput;

public class IntakeCommands {
  Intake intake;
  Map<String, GenericSubsystem> subsystems;
  StateMachine intakeStateMachine = new StateMachine("IntakeStateMachine");

  private static enum IntakeState {
    RETRACTED,
    RETRACTING,
    INTAKING,
    OUTTAKING
  }

  @AutoLogOutput(key = "IntakeCommands/RequestedIntakeState")
  private static IntakeState requestedState = IntakeState.RETRACTED;

  public IntakeCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;
    intake = (Intake) subsystems.get(Constants.INTAKE);
  }

  State retracted = intakeStateMachine.addState("retracted", retractedCommand());
  State outtaking = intakeStateMachine.addState("outtaking", outtakingCommand());
  State retracting = intakeStateMachine.addState("retracting", retractingCommand());
  State intaking = intakeStateMachine.addState("intaking", intakingCommand());

  public void configureButtonBindings(Controller controller) {
    controller.setRightTrigger(controller.createRightTrigger());
    Trigger rightTrigger = new Trigger(() -> controller.getRightTrigger() > 0.25);
    controller.setLeftTrigger(controller.createLeftTrigger());
    Trigger leftTrigger = new Trigger(() -> controller.getLeftTrigger() > 0.25);

    rightTrigger.onTrue(shouldIntaking()).onFalse(shouldRetracted());
    leftTrigger.onTrue(shouldOuttaking()).onFalse(shouldRetracted());
    retracting.switchTo(intaking).when(() -> requestedState == IntakeState.INTAKING);
    retracted.switchTo(intaking).when(() -> requestedState == IntakeState.INTAKING);
    intaking.switchTo(retracting).when(() -> requestedState == IntakeState.RETRACTING);
    retracting.switchTo(retracted).when(() -> intake.isRetracted());
  }

  public Command outtakingCommand() {
    return Commands.print("OUTTAKING")
        .andThen(() -> intake.RunSpintake(-25))
        .andThen(() -> intake.setPinionPosition(0));
    // assuming outtaking is just intaking but goes the other way
  }

  public Command intakingCommand() {
    return Commands.print("INTAKING")
        .andThen(() -> intake.RunSpintake(25))
        .andThen(() -> intake.setPinionPosition(0));
  }

  public Command retractingCommand() {
    return Commands.print("RETRACTING")
        .andThen(() -> intake.RunSpintake(0))
        .andThen(() -> intake.setPinionPosition(100));
  }

  public Command retractedCommand() {
    return Commands.print("RETRACTED")
        .andThen(() -> intake.RunSpintake(0))
        .andThen(() -> intake.setPinionPosition(0));
  }

  public Command shouldOuttaking() {
    return Commands.runOnce(() -> requestedState = IntakeState.OUTTAKING);
  }

  public Command shouldIntaking() {
    return Commands.runOnce(() -> requestedState = IntakeState.INTAKING);
  }

  public Command shouldRetracting() {
    return Commands.runOnce(() -> requestedState = IntakeState.RETRACTING);
  }

  public Command shouldRetracted() {
    return Commands.runOnce(() -> requestedState = IntakeState.RETRACTED);
  }
}
