package frc.robot.rebuilt.commands;

import static edu.wpi.first.units.Units.Degrees;

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

public class IntakeCommands {
  static Intake intake;
  Map<String, GenericSubsystem> subsystems;
  StateMachine intakeStateMachine = new StateMachine("IntakeStateMachine");
  State retracted = intakeStateMachine.addState("retracted", retractedCommand());
  State outtaking = intakeStateMachine.addState("outtaking", outtakingCommand());
  State retracting = intakeStateMachine.addState("retracting", retractingCommand());
  State intaking = intakeStateMachine.addState("intaking", intakingCommand());

  public static enum IntakeState {
    RETRACTED,
    RETRACTING,
    INTAKING,
    OUTTAKING;
  }

  public IntakeCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;

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
    controller.createLeftBumper().onTrue(shouldRetracting()).onFalse(shouldRetracted());

    rightTrigger.onTrue(shouldIntaking());
    leftTrigger.onTrue(shouldOuttaking());
    retracting.switchTo(intaking).when(() -> intake.isRequested(IntakeState.INTAKING));
    retracted.switchTo(intaking).when(() -> intake.isRequested(IntakeState.INTAKING));
    intaking.switchTo(retracting).when(() -> intake.isRequested(IntakeState.RETRACTING));
    intaking.switchTo(outtaking).when(() -> intake.isRequested(IntakeState.OUTTAKING));
    outtaking.switchTo(retracting).when(() -> intake.isRequested(IntakeState.RETRACTING));
    outtaking.switchTo(intaking).when(() -> intake.isRequested(IntakeState.INTAKING));
    retracting.switchTo(retracted).when(() -> intake.isRetracted());
  }

  public static Command outtakingCommand() {
    return Commands.runOnce(() -> intake.setCurrentState(IntakeState.OUTTAKING))
        .andThen(() -> intake.setHopperAngle(Degrees.of(0.0)))
        .andThen(() -> intake.runSpintake(-0.25));
    // assuming outtaking is just intaking but goes the other way
  }

  public static Command intakingCommand() {
    return Commands.runOnce(() -> intake.setCurrentState(IntakeState.INTAKING))
        .andThen(() -> intake.setHopperAngle(Degrees.of(0.0)))
        .andThen(() -> intake.runSpintake(0.25));
  }

  public static Command retractingCommand() {
    return Commands.runOnce(() -> intake.setCurrentState(IntakeState.RETRACTING))
        .andThen(() -> intake.setHopperAngle(Degrees.of(130)))
        .andThen(() -> intake.runSpintake(0));
  }

  public static Command retractedCommand() {
    return Commands.runOnce(() -> intake.setCurrentState(IntakeState.RETRACTED))
        .andThen(() -> intake.setHopperAngle(Degrees.of(130.0)))
        .andThen(() -> intake.runSpintake(0));
  }

  public static Command shouldOuttaking() {
    return Commands.runOnce(() -> intake.setRequestedState(IntakeState.OUTTAKING));
  }

  public static Command shouldIntaking() {
    return Commands.runOnce(() -> intake.setRequestedState(IntakeState.INTAKING));
  }

  public static Command shouldRetracting() {
    return Commands.runOnce(() -> intake.setRequestedState(IntakeState.RETRACTING));
  }

  public static Command shouldRetracted() {
    return Commands.runOnce(() -> intake.setRequestedState(IntakeState.RETRACTED));
  }
}
