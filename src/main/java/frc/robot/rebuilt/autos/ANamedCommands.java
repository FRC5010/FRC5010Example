package frc.robot.rebuilt.autos;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;

public class ANamedCommands {
  public static void CreateAutoNamedCommands() {
    NamedCommands.registerCommand("shoot 8", Commands.print("Fired 8"));
    NamedCommands.registerCommand("shoot", Commands.print("Fired"));
    NamedCommands.registerCommand("climb", Commands.print("climbed"));
  }
}
