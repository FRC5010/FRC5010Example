package frc.robot.rebuilt.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.subsystems.Indexer;
import frc.robot.rebuilt.subsystems.Intake;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.arch.StateMachine.State;

public class IntakeCommands {
  Indexer indexer;
  Intake intake;
  Map<String, GenericSubsystem> intakeMap;
  StateMachine intakeStateMachine = new StateMachine("IntakeStateMachine");

  public IntakeCommands(Map<String, GenericSubsystem> intakeMap) {
    this.intakeMap = intakeMap;
    indexer = new Indexer();
    intake = new Intake();
  }

  State retracting =
      intakeStateMachine.addState(
          "retracting", Commands.print("RETRACTING").andThen(intake.spintakeCommand(0)));
  State intaking = intakeStateMachine.addState("intaking", Commands.print("INTAKING").andThen());
}
