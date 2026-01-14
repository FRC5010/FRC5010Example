// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.rebuilt;
import org.frc5010.common.arch.StateMachine;
import org.frc5010.common.arch.StateMachine.State;
import org.frc5010.common.config.ConfigConstants;
import org.frc5010.common.constants.SwerveConstants;
import org.frc5010.common.drive.GenericDrivetrain;
import org.frc5010.common.motors.function.PercentControlMotor;
import org.frc5010.common.sensors.Controller;
import frc.robot.rebuilt.subsystems.Indexer;
import frc.robot.rebuilt.subsystems.Launcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Rebuilt extends SubsystemBase {
  SwerveConstants swerveConstants;
  GenericDrivetrain drivetrain;
  PercentControlMotor percentControlMotor;
  Indexer indexer;
  Launcher launcher;
  StateMachine stateMachine = new StateMachine("RebuiltStateMachine");
  /** Creates a new Rebuilt. */
  public Rebuilt() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
