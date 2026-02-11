package frc.robot.rebuilt.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import java.util.Map;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.velocity.FlyWheel;

public class IntakeIOReal implements IntakeIO {
  protected Map<String, Object> devices;
  private FlyWheel spintake;
  private Arm intakeHopper;

  public IntakeIOReal(Map<String, Object> devices) {
    this.devices = devices;
    spintake = (FlyWheel) devices.get("spintake");
    intakeHopper = (Arm) devices.get("hopper");
  }

  @Override
  public void runSpintake(double speed) {
    spintake.getMotorController().setDutyCycle(speed);
  }

  public void setHopperAngle(Angle angle) {
    intakeHopper.getMotorController().setPosition(angle);
  }

  public Boolean isRetracted() {
    return (intakeHopper.getAngle().isEquivalent(Degrees.of(0.0)));
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.hopperAngle = intakeHopper.getMotorController().getMechanismPosition();
    inputs.speed = spintake.getMotorController().getDutyCycle();
  }
}
