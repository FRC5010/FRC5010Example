package frc.robot.rebuilt.subsystems.intake;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double speed = 0.0;
    public Distance position = Meters.of(0.0);
    public int simulatedGamepieces = 0;
  }

  public void runSpintake(double speed);

  public void setPinionPosition(double position);

  public Boolean isRetracted();

  public default void updateInputs(IntakeIOInputs inputs) {}
}
