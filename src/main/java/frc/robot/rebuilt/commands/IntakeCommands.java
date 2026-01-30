package frc.robot.rebuilt.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.rebuilt.Constants;
import frc.robot.rebuilt.subsystems.Indexer.Indexer;
import frc.robot.rebuilt.subsystems.intake.Intake;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.arch.StateMachine.State;

public class IntakeCommands {
  Indexer indexer;
  Intake intake;
  Map<String, GenericSubsystem> subsystems;
  StateMachine intakeStateMachine = new StateMachine("IntakeStateMachine");

  public IntakeCommands(Map<String, GenericSubsystem> subsystems) {
    this.subsystems = subsystems;
    intake = (Intake) subsystems.get(Constants.INTAKE);
  }

  State retracting =
      intakeStateMachine.addState(
          "retracting",
          Commands.print("RETRACTING")
              .andThen(() -> intake.RunSpintake(0))
              .andThen(() -> intake.setHeight(100)));
  State intaking =
      intakeStateMachine.addState(
          "intaking",
          Commands.print("INTAKING")
              .andThen(() -> intake.RunSpintake(25))
              .andThen(() -> intake.setHeight(0)));
}
