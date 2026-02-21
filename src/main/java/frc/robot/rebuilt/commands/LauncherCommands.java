package frc.robot.rebuilt.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color;
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
import org.frc5010.common.subsystems.LEDStrip;

public class LauncherCommands {

  private StateMachine stateMachine;
  private State idleState;
  private State lowState;
  private State prepState;
  private State presetState;
  private State hammerTimeState;
  private static Launcher launcher;
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
    HAMMERTIME,
    PRESET;

    @Override
    public String toString() {
      return this.name();
    }
  }

  public LauncherCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;
    launcher = (Launcher) subsystems.get(Constants.LAUNCHER);
    launcher.setCurrentState(LauncherState.IDLE);
    launcher.setRequestedState(LauncherState.IDLE);

    drivetrain = (GenericDrivetrain) this.subsystems.get(ConfigConstants.DRIVETRAIN);

    stateMachine = new StateMachine("LauncherStateMachine");
    presetState = stateMachine.addState("PRESET-SHOOT", presetStateCommand());
    idleState = stateMachine.addState("IDLE", idleStateCommand());
    lowState = stateMachine.addState("LOW-SPEED", lowStateCommand());
    prepState = stateMachine.addState("PREP-SHOOT", prepStateCommand());
    hammerTimeState = stateMachine.addState("HAMMER-TIME", hammerTimeStateCommand());

    stateMachine.setInitialState(idleState);
  }

  public void setDefaultCommands() {
    if (launcher != null) {
      stateMachine.addRequirements(launcher);
      launcher.setDefaultCommand(stateMachine);
    }
  }

  public void configureButtonBindings(Controller driver, Controller operator) {

    idleState.switchTo(lowState).when(() -> launcher.isRequested(LauncherState.LOW_SPEED));
    idleState.switchTo(prepState).when(() -> launcher.isRequested(LauncherState.PREP));
    idleState.switchTo(presetState).when(() -> launcher.isRequested(LauncherState.PRESET));
    idleState.switchTo(hammerTimeState).when(() -> launcher.isRequested(LauncherState.HAMMERTIME));

    lowState.switchTo(idleState).when(() -> launcher.isRequested(LauncherState.IDLE));
    lowState.switchTo(prepState).when(() -> launcher.isRequested(LauncherState.PREP));
    lowState.switchTo(presetState).when(() -> launcher.isRequested(LauncherState.PRESET));
    lowState.switchTo(hammerTimeState).when(() -> launcher.isRequested(LauncherState.HAMMERTIME));

    prepState.switchTo(lowState).when(() -> launcher.isRequested(LauncherState.LOW_SPEED));
    prepState.switchTo(idleState).when(() -> launcher.isRequested(LauncherState.IDLE));
    prepState.switchTo(presetState).when(() -> launcher.isRequested(LauncherState.PRESET));
    prepState.switchTo(hammerTimeState).when(() -> launcher.isRequested(LauncherState.HAMMERTIME));

    presetState.switchTo(idleState).when(() -> launcher.isRequested(LauncherState.IDLE));
    presetState.switchTo(lowState).when(() -> launcher.isRequested(LauncherState.LOW_SPEED));
    presetState.switchTo(prepState).when(() -> launcher.isRequested(LauncherState.PREP));
    presetState
        .switchTo(hammerTimeState)
        .when(() -> launcher.isRequested(LauncherState.HAMMERTIME));

    // Hammer Time is a special case since it's a toggle state
    hammerTimeState.switchTo(lowState).when(() -> launcher.isRequested(LauncherState.LOW_SPEED));

    driver.createAButton().onTrue(shouldLowCommand());

    driver.createBButton().onTrue(shouldHammerTimeCommand());

    operator.createLeftBumper().whileTrue(shouldPrepCommand()).onFalse(shouldLowCommand());

    operator.createAButton().whileTrue(towerPresetStateCommand()).onFalse(shouldIdleCommand());

    operator.createBButton().onTrue(shouldHammerTimeCommand());

    operator.createXButton().whileTrue(hubPresetStateCommand()).onFalse(shouldIdleCommand());
    operator
        .createYButton()
        .whileTrue(turretForwardPresetStateCommand())
        .onFalse(shouldIdleCommand());

    Trigger readyToFireTrigger =
        new Trigger(() -> launcher.isCurrent(LauncherState.PREP) && launcher.isAtGoal());
    readyToFireTrigger
        .onTrue(IndexerCommands.shouldFeedCommand())
        .onFalse(IndexerCommands.shouldIdleCommand());
  }

  private Translation2d getTargetPose() {
    return target.minus(drivetrain.getPoseEstimator().getCurrentPose().getTranslation());
  }

  private static Command idleStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              launcher.setCurrentState(LauncherState.IDLE);
            }),
        launcher.stopTrackingCommand());
  }

  private static Command lowStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              launcher.setCurrentState(LauncherState.LOW_SPEED);
              LEDStrip.changeSegmentPattern(
                  ConfigConstants.ALL_LEDS, LEDStrip.getSolidPattern(Color.kGreen));
            }),
        launcher.trackTargetCommand());
  }

  private static Command prepStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              launcher.setCurrentState(LauncherState.PREP);
              LEDStrip.changeSegmentPattern(
                  ConfigConstants.ALL_LEDS, LEDStrip.getRainbowPattern(0));
            }),
        launcher.trackTargetCommand());
  }

  private static Command presetStateCommand() {
    return Commands.parallel(
        Commands.runOnce(
            () -> {
              launcher.setCurrentState(LauncherState.PRESET);
            }));
  }

  public static Command shouldIdleCommand() {
    return Commands.runOnce(() -> launcher.setRequestedState(LauncherState.IDLE));
  }

  public static Command shouldLowCommand() {
    return Commands.runOnce(() -> launcher.setRequestedState(LauncherState.LOW_SPEED));
  }

  public static Command shouldPrepCommand() {
    return Commands.runOnce(() -> launcher.setRequestedState(LauncherState.PREP));
  }

  public static Command shouldPresetCommand() {
    return Commands.runOnce(() -> launcher.setRequestedState(LauncherState.PRESET));
  }

  public static Command shouldHammerTimeCommand() {
    return Commands.runOnce(
        () -> {
          if (launcher.getCurrentState() == LauncherState.HAMMERTIME) {
            launcher.setRequestedState(LauncherState.LOW_SPEED);
          } else {
            launcher.setRequestedState(LauncherState.HAMMERTIME);
          }
        });
  }

  // Order is Hood Angle, Turret Angle, Flywheel Speed
  // Values are placeholders and need to be tuned
  public static Command hubPresetStateCommand() {
    return shouldPresetCommand()
        .andThen(
            Commands.runOnce(
                () ->
                    launcher.usePresets(
                        Degrees.of(30), Constants.LauncherConstants.TURRET_FORWARD, RPM.of(300))));
  }

  public static Command towerPresetStateCommand() {
    return shouldPresetCommand()
        .andThen(
            Commands.runOnce(
                () ->
                    launcher.usePresets(
                        Degrees.of(45), Constants.LauncherConstants.TURRET_FORWARD, RPM.of(400))));
  }

  public static Command turretForwardPresetStateCommand() {
    return shouldPresetCommand()
        .andThen(
            Commands.runOnce(
                () ->
                    launcher.usePresets(
                        Degrees.of(50), Constants.LauncherConstants.TURRET_FORWARD, RPM.of(500))));
  }

  public static Command hammerTimeStateCommand() {
    return Commands.parallel(
        Commands.run(
            () -> {
              launcher.setCurrentState(LauncherState.HAMMERTIME);
              launcher.usePresets(
                  Constants.LauncherConstants.LOW_HOOD_ANGLE,
                  Degrees.of(90),
                  Constants.LauncherConstants.LOW_FLYWHEEL_RPM);
            }));
  }

  private boolean isNotHammerTime() {
    return !launcher.isCurrent(LauncherState.HAMMERTIME);
  }

  public static LauncherState getCurrentState() {
    return launcher.getCurrentState();
  }
}
