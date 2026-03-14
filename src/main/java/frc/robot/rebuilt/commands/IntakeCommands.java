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
import org.frc5010.common.arch.StateMachine.State;
import org.frc5010.common.sensors.Controller;

public class IntakeCommands {
  static Intake intake;
  Map<String, GenericSubsystem> subsystems;
  StateMachine intakeStateMachine = new StateMachine("IntakeStateMachine");
  /** Declaring states of the intake */
  State unknown;

  State retracted;

  State retracting;
  State deploying;
  State deployed;
  State intaking;

  DoubleSupplier intakeSpeedSupplier = () -> 0.5;
  Supplier<DoubleSupplier> intakeSpeed = () -> intakeSpeedSupplier;

  public static enum IntakeState {
    UNKNOWN,
    RETRACTED,
    RETRACTING,
    DEPLOYING,
    INTAKING,
    DEPLOYED;
  }

  public IntakeCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;

    intake = (Intake) subsystems.get(Constants.INTAKE);

    //    setupStateMachine();
    setupTriggerStates();
  }

  public void setupDefaultCommands() {
    // intake.setDefaultCommand(retractedCommand());
  }

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

    new Trigger(
            () ->
                intake.isHopperAtGoal()
                    && intake.isRequested(IntakeState.INTAKING)
                    && intake.isCurrent(IntakeState.DEPLOYING)
                    && intake.getHopperAngle().lt(Degrees.of(20)))
        .onTrue(intakingCommand(intakeSpeed));

    new Trigger(
            () ->
                intake.isRequested(IntakeState.INTAKING) && !intake.isCurrent(IntakeState.INTAKING))
        .onTrue(deployingCommand());
    new Trigger(
            () ->
                intake.isRequested(IntakeState.RETRACTING)
                    && !intake.isCurrent(IntakeState.RETRACTED))
        .onTrue(retractingCommand());
    new Trigger(
            () ->
                intake.isCurrent(IntakeState.RETRACTING)
                    && intake.isHopperAtGoal()
                    && intake.getHopperAngle().gt(Degrees.of(50)))
        .onTrue(shouldRetracted());
  }

  private void setupStateMachine() {
    unknown = intakeStateMachine.addState("unknown", unknownStateCommand());
    // For auto - these get replaced with versions that take controller input instead of constants
    retracted = intakeStateMachine.addState("retracted", retractedCommand());
    retracting = intakeStateMachine.addState("retracting", retractingCommand());
    deploying = intakeStateMachine.addState("deploying", deployingCommand());
    intaking = intakeStateMachine.addState("intaking", intakingCommand(intakeSpeed));

    // If the hopper is not moving and we want to intake or outtake, set the hopper angle to 0 to
    // prevent jamming
    // hopperNotMoving
    //     .and(() -> intake.isRequested(IntakeState.INTAKING))
    //     .onTrue(Commands.runOnce(() -> intake.setHopperPosition(Degrees.of(0))));
    // hopperNotMoving
    //     .and(() -> intake.isRequested(IntakeState.OUTTAKING))
    //     .onTrue(Commands.runOnce(() -> intake.setHopperPosition(Degrees.of(0))));

    // Get out of the unknown state when we deploy so it hits the hardstop
    unknown.switchTo(deploying).when(() -> intake.isRequested(IntakeState.INTAKING));

    deployed.switchTo(retracting).when(() -> intake.isRequested(IntakeState.RETRACTING));
    deployed.switchTo(intaking).when(() -> intake.isRequested(IntakeState.INTAKING));

    retracted.switchTo(deploying).when(() -> intake.isRequested(IntakeState.INTAKING));
    deploying
        .switchTo(intaking)
        .when(() -> (intake.isHopperAtGoal()) && intake.isRequested(IntakeState.INTAKING));

    retracting.switchTo(deploying).when(() -> intake.isRequested(IntakeState.INTAKING));

    intaking.switchTo(retracting).when(() -> intake.isRequested(IntakeState.RETRACTING));

    retracting.switchTo(retracted).when(() -> intake.isRetracted());
    deployed.switchTo(retracted).when(() -> intake.isRetracted());

    deploying.switchTo(deployed).when(() -> intake.isRequested(IntakeState.DEPLOYED));
    intaking.switchTo(deployed).when(() -> intake.isRequested(IntakeState.DEPLOYED));
    retracted.switchTo(deployed).when(() -> intake.isRequested(IntakeState.DEPLOYED));
    retracting.switchTo(deployed).when(() -> intake.isRequested(IntakeState.DEPLOYED));

    intakeStateMachine.setInitialState(unknown);
    if (intake != null) {
      intakeStateMachine.addRequirements(intake);
    }
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

    // Not moving trigger senses if the hopper has hit the bumper hard stop for 0.5 sec
    /*   Trigger hopperNotMoving =
            new Trigger(
                    () ->
                        intake.getCurrentState() == IntakeState.DEPLOYING && intake.isHopperStalling())
                .debounce(0.5)
                .onTrue(Commands.runOnce(() -> intake.setHopperPosition(Degrees.of(0))));
    */
    controller.createRightBumper().onTrue(shouldRetracting()).onFalse(shouldRetracted());

    controller.createStartButton().onTrue(Commands.run(() -> intake.setHopperRetracted()));
    controller.createBackButton().onTrue(Commands.run(() -> intake.setHopperDeployed()));

    intakeSpeedSupplier =
        () -> {
          return controller.getRightTrigger();
        };
  }

  public static Command intakingCommand(Supplier<DoubleSupplier> speed) {
    return Commands.runOnce(
            () -> {
              intake.setCurrentState(IntakeState.INTAKING);
            },
            intake)
        // .andThen(Commands.runOnce(() -> intake.runHopper(0)))
        // .andThen(() -> intake.setHopperAngle(Degrees.of(0.0)))
        .andThen(Commands.run(() -> intake.runSpintake(speed.get().getAsDouble()), intake));
    // Math.min(
    //     Constants.Intake.INTAKE_MAX_IN,
    //     Math.max(speed.getAsDouble(), Constants.Intake.INTAKE_IN)))));
  }

  public static Command deployingCommand() {
    return Commands.runOnce(() -> intake.setCurrentState(IntakeState.DEPLOYING), intake)
        .andThen(intake.setDesiredHopperAngle(Degrees.of(0)).until(() -> intake.isHopperAtGoal()))
        .andThen(Commands.idle(intake));
  }

  public static Command deployedCommand() {
    return Commands.run(
        () -> {
          intake.setCurrentState(IntakeState.DEPLOYED);
          intake.runHopper(0);
        },
        intake);
  }

  public static Command retractingCommand() {
    return Commands.runOnce(
            () -> {
              intake.setCurrentState(IntakeState.RETRACTING);
              LauncherCommands.shouldIdleCommand();
            },
            intake)
        .andThen(() -> intake.runSpintake(0), intake)
        //        .andThen(() -> intake.runHopper(Constants.Intake.HOPPER_GO_IN))
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
}
