package frc.robot.rebuilt.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;

import frc.robot.rebuilt.Rebuilt;
import java.util.Map;
import org.frc5010.common.drive.GenericDrivetrain;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import swervelib.simulation.ironmaple.simulation.gamepieces.GamePieceOnFieldSimulation;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

public class IntakeIOSim extends IntakeIOReal {
  public static IntakeSimulation intakeSimulation;
  private AbstractDriveTrainSimulation driveTrainSimulation;
  private GamePieceOnFieldSimulation gamePiece;

  public IntakeIOSim(Map<String, Object> devices) {
    super(devices);
    driveTrainSimulation = GenericDrivetrain.getMapleSimDrive().get();
    intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            driveTrainSimulation,
            Inches.of(27.25),
            Inches.of(11.25),
            IntakeSimulation.IntakeSide.FRONT,
            80);
  }

  @Override
  public void runSpintake(double speed) {
    super.runSpintake(speed);
    if (speed > 0) {
      intakeSimulation.startIntake();
    } else {
      intakeSimulation.stopIntake();
    }
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    super.updateInputs(inputs);
    if (inputs.speed < 0) {
      if (intakeSimulation.obtainGamePieceFromIntake()) {
        gamePiece =
            new RebuiltFuelOnField(
                Rebuilt.drivetrain.getPoseEstimator().getCurrentPose().getTranslation());
        SimulatedArena.getInstance().addGamePiece(gamePiece);
      }
    }
    inputs.simulatedGamepieces = intakeSimulation.getGamePiecesAmount();
  }
}
