package frc.robot.commands.FlywheelCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Flywheel;

public class DefaultFlywheel extends Command{
    
    Flywheel hood;

    public 
    DefaultFlywheel(Flywheel hood) {
        this.hood = hood;
        addRequirements(hood);
    }

   public void execute(){
        RobotContainer.shooting = false; 
        hood.stopMotors();
        //WALLAHIIGOHGJPIEOIOIEGWOEGOEGOWEG:GW:GW:HWEG:HEGHUEGOHGTWHTGWGHUGTHUP
    }
}
