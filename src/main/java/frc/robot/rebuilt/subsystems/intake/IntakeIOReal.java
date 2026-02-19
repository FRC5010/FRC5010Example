package frc.robot.rebuilt.subsystems.intake;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Map;
import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.motors.SystemIdentification;
import org.frc5010.common.motors.function.PercentControlMotor;
import yams.mechanisms.positional.Arm;

public class IntakeIOReal implements IntakeIO {
  protected Map<String, Object> devices;
  // private FlyWheel spintake;
  private PercentControlMotor spinTakeLead;
  private PercentControlMotor spinTakeFollow;
  private Arm intakeHopper;

  public IntakeIOReal(Map<String, Object> devices) {
    this.devices = devices;
    // spintake = (FlyWheel) devices.get("spintake");
    spinTakeLead = (PercentControlMotor) devices.get("spintakeLead");
    spinTakeFollow = (PercentControlMotor) devices.get("spintakeFollow");
    spinTakeLead.setFollow(spinTakeFollow, true);
    intakeHopper = (Arm) devices.get("hopper");
  }

  @Override
  public void runSpintake(double speed) {
    spinTakeLead.set(speed);
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

  public Command getHopperSysIdCommand(GenericSubsystem intake) {
    return SystemIdentification.getSysIdFullCommand(
        SystemIdentification.angleSysIdRoutine(
            intakeHopper.getMotorController(), intakeHopper.getName(), intake),
        5,
        5,
        3,
        () ->
            intakeHopper
                .isNear(
                    intakeHopper.getMotorController().getConfig().getMechanismUpperLimit().get(),
                    Degrees.of(10))
                .getAsBoolean(),
        () ->
            intakeHopper
                .isNear(
                    intakeHopper.getMotorController().getConfig().getMechanismLowerLimit().get(),
                    Degrees.of(10))
                .getAsBoolean(),
        () -> intakeHopper.getMotor().setDutyCycle(0));
  }

  public Command getHopperCharacterizationCommand(GenericSubsystem intake) {
    return SystemIdentification.feedforwardCharacterization(
        intake,
        (Voltage voltage) -> intakeHopper.getMotor().setVoltage(voltage),
        () -> intakeHopper.getMotorController().getMechanismVelocity().in(Degrees.per(Second)));
  }

  public void runHopper(double speed) {
    intakeHopper.getMotorController().setDutyCycle(speed);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.hopperAngle = intakeHopper.getMotorController().getMechanismPosition();
    inputs.speed = spinTakeLead.get();
  }
}
