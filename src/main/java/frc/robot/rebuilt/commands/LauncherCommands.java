package frc.robot.rebuilt.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private DisplayString commandState;
  private DisplayValuesHelper DisplayHelper;
  private State idleState;
  private State lowState;
  private State prepState;
  private State readyState;
  private Launcher launcher;
  private GenericDrivetrain drivetrain;
  private Map<String, GenericSubsystem> subsystems;
  private Translation2d target = new Translation2d(Inches.of(182.11), Inches.of(158.84));

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

  public void configureButtonBindings(Controller controller) {

    Trigger rightBumper = controller.createRightBumper();
    Trigger leftBumper = controller.createLeftBumper();

    lowState.switchTo(prepState).when(rightBumper);
    prepState.switchTo(lowState).when(() -> !rightBumper.getAsBoolean());

    prepState.switchTo(readyState).when(leftBumper);
    readyState.switchTo(lowState).when(() -> !leftBumper.getAsBoolean());

    // if (lowState != null && lowState.isScheduled()) {

    //   launcher.setHoodAngle(Units.Degrees.of(0));
    //   launcher.setLowerSpeed(0.5);
    //   launcher.setTurretRotation(Units.Degrees.of(0));
    // }
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
}
