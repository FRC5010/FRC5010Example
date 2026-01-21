package frc.robot.rebuilt.commands;

import java.util.Map;

import org.frc5010.common.arch.GenericSubsystem;

public class AutoCommands {

    private Map<String, GenericSubsystem> subsystems;
    
    public void AutoCommands(Map<String, GenericSubsystem> subsystems){
    this.subsystems = subsystems;
}
    

 public static void configureNamedCommands(){

 }

}

