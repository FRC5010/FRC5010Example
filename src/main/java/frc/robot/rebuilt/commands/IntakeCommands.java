package frc.robot.rebuilt.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.subsystems.intake.Intake;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.sensors.Controller;

public class IntakeCommands {
  static Intake intake;
  Map<String, GenericSubsystem> subsystems;
  StateMachine intakeStateMachine = new StateMachine("IntakeStateMachine");

  DoubleSupplier intakeSpeedSupplier = () -> 0.5;
  Supplier<DoubleSupplier> intakeSpeed = () -> intakeSpeedSupplier;

  public static enum IntakeState {
    UNKNOWN,
    RETRACTED,
    RETRACTING,
    DEPLOYING,
    INTAKING,
    DEPLOYED,
    ANGLED;
  }

  public IntakeCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;

    intake = (Intake) subsystems.get(Constants.INTAKE);

    setupTriggerStates();
  }

  public void setupDefaultCommands() {}

  private void setupTriggerStates() {
    // Map requested states to their commands and wire triggers in a compact loop.
    // CHURN is handled separately below so it can be gated on flywheel readiness.
    java.util.Map<IntakeState, Command> stateToCommand =
        java.util.Map.of(
            IntakeState.UNKNOWN, unknownStateCommand(),
            IntakeState.RETRACTED, retractedCommand(),
            IntakeState.DEPLOYED, deployedCommand());

    stateToCommand.forEach(
        (state, cmd) -> new Trigger(() -> intake.isRequested(state)).onTrue(cmd));

    /** Trigger the deploying command */
    new Trigger(
            () ->
                intake.isRequested(IntakeState.INTAKING) && !intake.isCurrent(IntakeState.INTAKING))
        .onTrue(deployingCommand());

    /** Trigger the intaking command */
    new Trigger(
            () ->
                intake.isRequested(IntakeState.INTAKING)
                    && intake.isHopperAtGoal()
                    && intake.isCurrent(IntakeState.DEPLOYING)
                    && intake.getHopperAngle().lt(Degrees.of(10)))
        .onTrue(intakingCommand(intakeSpeed));

    /** Trigger the retracting command */
    new Trigger(
            () ->
                intake.isRequested(IntakeState.RETRACTING)
                    && !intake.isCurrent(IntakeState.RETRACTED))
        .onTrue(retractingCommand());

    /** Trigger the retracted command */
    new Trigger(
            () ->
                intake.isCurrent(IntakeState.RETRACTING)
                    && intake.isHopperAtGoal()
                    && intake.getHopperAngle().gt(Degrees.of(120)))
        .onTrue(shouldRetracted());

    /** Trigger the angled command */
    new Trigger(
            () -> intake.isRequested(IntakeState.ANGLED) && !intake.isCurrent(IntakeState.ANGLED))
        .onTrue(angledCommand());
  }

  public void configureButtonBindings(Controller controller) {
    controller.setRightTrigger(
        controller.createRightTrigger().limit(Constants.Intake.INTAKE_MAX_IN));
    Trigger rightTrigger = new Trigger(() -> controller.getRightTrigger() > 0.25);
    controller.setLeftTrigger(
        controller
            .createLeftTrigger()
            .limit(Constants.Intake.INTAKE_MAX_IN)); // Axis are positive only hence IN
    Trigger leftTrigger = new Trigger(() -> controller.getLeftTrigger() > 0.25);

    rightTrigger.onTrue(shouldIntaking());
    leftTrigger.onTrue(shouldIntaking());

    controller.createRightBumper().onTrue(shouldRetracting());
    controller.createLeftBumper().onTrue(shouldAngled());
    controller.createStartButton().onTrue(Commands.run(() -> intake.setHopperRetracted()));
    controller.createBackButton().onTrue(Commands.run(() -> intake.setHopperDeployed()));

    intakeSpeedSupplier =
        () -> {
          double rightTriggerSpeed = controller.getRightTrigger();
          double leftTriggerSpeed = controller.getLeftTrigger();
          double speed = 0.5;
          if (rightTriggerSpeed > 0.25 || leftTriggerSpeed > 0.25) {
            speed =
                rightTriggerSpeed
                    - leftTriggerSpeed; // Positive for intaking, negative for outtaking
          }
          return speed;
        };
  }

  public static Command intakingCommand(Supplier<DoubleSupplier> speed) {
    return Commands.runOnce(
            () -> {
              intake.setCurrentState(IntakeState.INTAKING);
            },
            intake)
        .andThen(Commands.run(() -> intake.runSpintake(speed.get().getAsDouble()), intake));
  }

  public static Command deployingCommand() {
    return Commands.runOnce(() -> intake.setCurrentState(IntakeState.DEPLOYING), intake)
        .andThen(intake.setDesiredHopperAngle(Degrees.of(0)).until(() -> intake.isHopperAtGoal()))
        .andThen(Commands.idle(intake));
  }

  public static Command deployedCommand() {
    return Commands.runOnce(
        () -> {
          intake.setCurrentState(IntakeState.DEPLOYED);
          intake.runHopper(0);
        },
        intake);
  }

  public static Command angledCommand() {
    return Commands.runOnce(
            () -> {
              intake.setCurrentState(IntakeState.ANGLED);
            },
            intake)
        .andThen(intake.setDesiredHopperAngle(Degrees.of(45)).until(() -> intake.isHopperMoving()));
  }

  public static Command retractingCommand() {
    return Commands.runOnce(
            () -> {
              intake.setCurrentState(IntakeState.RETRACTING);
              LauncherCommands.shouldIdleCommand();
            },
            intake)
        .andThen(() -> intake.runSpintake(0), intake)
        .andThen(
            intake.setDesiredHopperAngle(Degrees.of(130.0)).until(() -> intake.isHopperMoving()));
  }

  public static Command retractedCommand() {
    return Commands.runOnce(() -> intake.setCurrentState(IntakeState.RETRACTED))
        .andThen(() -> intake.runSpintake(0), intake)
        .andThen(() -> intake.runHopper(0), intake);
  }

  public static Command unknownStateCommand() {
    return Commands.runOnce(
            () -> {
              intake.setCurrentState(IntakeState.UNKNOWN);
            })
        .andThen(Commands.runOnce(() -> intake.runHopper(0), intake))
        .andThen(Commands.runOnce(() -> intake.runSpintake(0), intake));
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

  public static Command shouldAngled() {
    return Commands.runOnce(() -> intake.setRequestedState(IntakeState.ANGLED));
  }
}
