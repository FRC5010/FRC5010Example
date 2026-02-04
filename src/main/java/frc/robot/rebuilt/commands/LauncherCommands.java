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

public class LauncherCommands {

  private StateMachine stateMachine;
  private DisplayValuesHelper Dashboard;
  private static DisplayString currentState;
  private static DisplayString requestedState;
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

  public static enum LauncherState {
    IDLE,
    LOW_SPEED,
    PREP,
    READY;

    @Override
    public String toString() {
      return this.name();
    }
  }

  public LauncherCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;
    Dashboard = new DisplayValuesHelper("LauncherCommands");
    currentState = Dashboard.makeDisplayString("CurrentState");
    requestedState = Dashboard.makeDisplayString("RequestedState");

    setCurrentState(LauncherState.IDLE);
    setRequestedState(LauncherState.IDLE);

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

    idleState.switchTo(lowState).when(() -> isRequested(LauncherState.LOW_SPEED));
    idleState.switchTo(prepState).when(() -> isRequested(LauncherState.PREP));

    lowState.switchTo(idleState).when(() -> isRequested(LauncherState.IDLE));
    lowState.switchTo(prepState).when(() -> isRequested(LauncherState.PREP));

    prepState.switchTo(idleState).when(() -> isRequested(LauncherState.IDLE));
    prepState.switchTo(lowState).when(() -> isRequested(LauncherState.LOW_SPEED));
    // prepState.switchTo(readyState).when(() -> isRequested(LauncherState.READY)); PLACEHOLDER
    // FOR NOW

    readyState.switchTo(idleState).when(() -> isRequested(LauncherState.IDLE));
    readyState.switchTo(lowState).when(() -> isRequested(LauncherState.LOW_SPEED));
    // readyState.switchTo(prepState).when(() -> isRequested(LauncherState.PREP)); PLACEHOLDER
    // FOR NOW

  }

  public boolean isRequested(LauncherState state) {
    return LauncherState.valueOf(requestedState.getValue()) == state;
  }

  public boolean isCurrent(LauncherState state) {
    return LauncherState.valueOf(currentState.getValue()) == state;
  }

  private void setCurrentState(LauncherState state) {
    currentState.setValue(state.name());
  }

  private void setRequestedState(LauncherState state) {
    requestedState.setValue(state.name());
  }

  private Translation2d getTargetPose() {
    return target.minus(drivetrain.getPoseEstimator().getCurrentPose().getTranslation());
  }

  private Command idleStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              setCurrentState(LauncherState.IDLE);
            }),
        launcher.stopTrackingCommand());
  }

  private Command lowStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              setCurrentState(LauncherState.LOW_SPEED);
            }),
        launcher.trackTargetCommand());
  }

  private Command prepStateCommand() {
    return Commands.parallel(
        Commands.print("Launcher in PREP state"),
        Commands.runOnce(
            () -> {
              setCurrentState(LauncherState.PREP);
            }),
        launcher.trackTargetCommand());
  }

  private Command readyStateCommand() {
    return Commands.parallel(
        Commands.print("Launcher in READY state"),
        Commands.runOnce(
            () -> {
              setCurrentState(LauncherState.READY);
            }),
        launcher.trackTargetCommand());
  }

  public Command shouldIdleCommand() {
    return Commands.runOnce(() -> setRequestedState(LauncherState.IDLE));
  }

  public Command shouldLowCommand() {
    return Commands.runOnce(() -> setRequestedState(LauncherState.LOW_SPEED));
  }

  public Command shouldPrepCommand() {
    return Commands.runOnce(() -> setRequestedState(LauncherState.PREP));
  }

  public Command shouldReadyCommand() {
    return Commands.runOnce(() -> setRequestedState(LauncherState.READY));
  }

  public static LauncherState getCurrentState() {
    return LauncherState.valueOf(currentState.getValue());
  }
}
