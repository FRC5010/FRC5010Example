package frc.robot.rebuilt.subsystems.intake;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import java.util.Map;
import yams.mechanisms.positional.Elevator;
import yams.mechanisms.velocity.FlyWheel;

public class IntakeIOReal implements IntakeIO {
  protected Map<String, Object> devices;
  private FlyWheel spintake;
  private Elevator intakePinion;

  public IntakeIOReal(Map<String, Object> devices) {
    this.devices = devices;
    spintake = (FlyWheel) devices.get("spintake");
    intakePinion = (Elevator) devices.get("pinion");
  }

  @Override
  public void runSpintake(double speed) {
    spintake.getMotorController().setDutyCycle(speed);
  }

  public void setPinionPosition(double position) {
    Distance mydist = Meters.of(position);
    intakePinion.getMotorController().setPosition(mydist);
  }

  public Boolean isRetracted() {
    return (intakePinion.getHeight().isEquivalent(Inches.of(0)));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.position = intakePinion.getMotorController().getMeasurementPosition();
    inputs.speed = spintake.getMotorController().getDutyCycle();
  }
}
