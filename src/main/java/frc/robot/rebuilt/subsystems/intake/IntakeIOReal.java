package frc.robot.rebuilt.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
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

  public Command getHopperSysIdCommand() {
    return intakeHopper.sysId(Volts.of(4), Volts.of(0.5).per(Seconds), Seconds.of(8));
  }

  public void runHopper(double speed) {
    intakeHopper.getMotorController().setDutyCycle(speed);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.hopperAngle = intakeHopper.getMotorController().getMechanismPosition();
    inputs.speed = spintake.getMotorController().getDutyCycle();
  }
}
