package frc.robot.rebuilt.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.subsystems.Launcher.Launcher;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.arch.StateMachine.State;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.sensors.Controller;
import org.frc5010.common.telemetry.DisplayString;
import org.frc5010.common.telemetry.DisplayValuesHelper;
import org.littletonrobotics.junction.AutoLogOutput;

public class LauncherCommands {

  private StateMachine stateMachine;
  private DisplayString commandState;
  private DisplayValuesHelper DisplayHelper;
  private State idleState;
  private State lowState;
  private State prepState;
  private State readyState;
  private Launcher launcher;
  private static GenericDrivetrain drivetrain;
  private Map<String, GenericSubsystem> subsystems;
  private static Translation2d target = new Translation2d(Inches.of(182.11), Inches.of(158.84));

  public static Translation2d getRobotToTarget() {
    return target.minus(drivetrain.getPoseEstimator().getCurrentPose().getTranslation());
  }
  // public static Angle getHoodAngle(Distance toTarget) {} Placeholder for now

  private static enum LauncherState {
    IDLE,
    LOW_SPEED,
    PREP,
    READY
  }

  @AutoLogOutput(key = "LauncherCommands/RequestedLauncherState")
  private static LauncherState requestedState = LauncherState.IDLE;

  public LauncherCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;
    DisplayHelper = new DisplayValuesHelper("LauncherCommands", "Values");
    commandState = DisplayHelper.makeDisplayString("Launcher State");

    launcher = (Launcher) subsystems.get(Constants.LAUNCHER);
    drivetrain = (GenericDrivetrain) this.subsystems.get(ConfigConstants.DRIVETRAIN);

    stateMachine = new StateMachine("LauncherStateMachine");
    idleState = stateMachine.addState("IDLE", idleStateCommand());
    lowState = stateMachine.addState("LOW-SPEED", lowStateCommand());
    prepState = stateMachine.addState("PREP-SHOOT", prepStateCommand());
    readyState = stateMachine.addState("READY-TO-SHOOT", readyStateCommand());
    stateMachine.setInitialState(lowState);
  }

  public void setDefaultCommands() {
    if (launcher != null) {
      stateMachine.addRequirements(launcher);
      launcher.setDefaultCommand(stateMachine);
    }
  }

  public void configureButtonBindings(Controller driver, Controller operator) {

    driver.createRightBumper().onTrue(shouldPrepCommand()).onFalse(shouldIdleCommand());

    idleState.switchTo(lowState).when(() -> requestedState == LauncherState.LOW_SPEED);
    idleState.switchTo(prepState).when(() -> requestedState == LauncherState.PREP);

    lowState.switchTo(idleState).when(() -> requestedState == LauncherState.IDLE);
    lowState.switchTo(prepState).when(() -> requestedState == LauncherState.PREP);

    prepState.switchTo(idleState).when(() -> requestedState == LauncherState.IDLE);
    prepState.switchTo(lowState).when(() -> requestedState == LauncherState.LOW_SPEED);
    // prepState.switchTo(readyState).when(() -> requestedState == LauncherState.READY); PLACEHOLDER
    // FOR NOW

    readyState.switchTo(idleState).when(() -> requestedState == LauncherState.IDLE);
    readyState.switchTo(lowState).when(() -> requestedState == LauncherState.LOW_SPEED);
    // readyState.switchTo(prepState).when(() -> requestedState == LauncherState.PREP); PLACEHOLDER
    // FOR NOW

  }

  private Translation2d getTargetPose() {
    return target.minus(drivetrain.getPoseEstimator().getCurrentPose().getTranslation());
  }

  private Command idleStateCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> commandState.setValue("Idle")), launcher.stopTrackingCommand());
  }

  private Command lowStateCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> commandState.setValue("Low Speed")),
        launcher.trackTargetCommand(() -> getTargetPose()));
  }

  private Command prepStateCommand() {
    return Commands.parallel(
        Commands.print("Launcher in PREP state"),
        Commands.runOnce(() -> commandState.setValue("Prep")),
        launcher.trackTargetCommand(() -> getTargetPose()));
  }

  private Command readyStateCommand() {
    return Commands.parallel(
        Commands.print("Launcher in READY state"),
        Commands.runOnce(() -> commandState.setValue("Ready")),
        launcher.trackTargetCommand(() -> getTargetPose()));
  }

  public Command shouldIdleCommand() {
    return Commands.runOnce(() -> requestedState = LauncherState.IDLE);
  }

  public Command shouldLowCommand() {
    return Commands.runOnce(() -> requestedState = LauncherState.LOW_SPEED);
  }

  public Command shouldPrepCommand() {
    return Commands.runOnce(() -> requestedState = LauncherState.PREP);
  }

  public Command shouldReadyCommand() {
    return Commands.runOnce(() -> requestedState = LauncherState.READY);
  }
}
