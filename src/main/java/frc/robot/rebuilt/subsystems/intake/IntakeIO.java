package frc.robot.rebuilt.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import frc.robot.rebuilt.commands.IntakeCommands;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public IntakeCommands.IntakeState stateRequested = IntakeCommands.IntakeState.RETRACTED;
    public IntakeCommands.IntakeState stateCurrent = IntakeCommands.IntakeState.RETRACTED;
    public double speed = 0.0;
    public Angle hopperAngle = Degrees.of(0.0);
    public int simulatedGamepieces = 0;
  }

  public void runSpintake(double speed);

  public void setHopperAngle(Angle angle);

  public Boolean isRetracted();

  public default void updateInputs(IntakeIOInputs inputs) {}
}
