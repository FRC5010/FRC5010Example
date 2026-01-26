package frc.robot.rebuilt.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import java.util.Map;
import org.frc5010.common.drive.GenericDrivetrain;
import swervelib.simulation.ironmaple.simulation.IntakeSimulation;
import swervelib.simulation.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

public class IntakeIOSim extends IntakeIOReal {
  public static IntakeSimulation intakeSimulation;
  private AbstractDriveTrainSimulation driveTrainSimulation;

  public IntakeIOSim(Map<String, Object> devices) {
    super(devices);
    driveTrainSimulation = GenericDrivetrain.getMapleSimDrive().get();
    intakeSimulation =
        IntakeSimulation.OverTheBumperIntake(
            "Gamepiece",
            driveTrainSimulation,
            Meters.of(0.7),
            Meters.of(0.2),
            IntakeSimulation.IntakeSide.BACK,
            1);
  }
}
