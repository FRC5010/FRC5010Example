package frc.robot.rebuilt.commands;

import java.util.Map;

import org.frc5010.common.arch.GenericSubsystem;
import org.frc5010.common.sensors.Controller;

public class ClimbCommands {

    private Map<String, GenericSubsystem> subsystems;
    
    public void ClimbCommands(Map<String, GenericSubsystem> subsystems){
    this.subsystems = subsystems;
}
    

 public void configureButtonBindings(Controller controller){

 }

}

