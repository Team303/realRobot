package frc.robot.commands.IntakeBeltCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeBelt;

public class IntakeStuff extends Command {
    
    boolean forward; 
    IntakeBelt intakeBelt;

    public 
    IntakeStuff(IntakeBelt intakebelt, boolean forward) {
        addRequirements(intakebelt);
        this.intakeBelt = intakebelt;
        this.forward = forward;
    }
    public void initialize(){
        
    }
    public void execute(){
       
        if (forward) {
            intakeBelt.beltMotor.set(-0.50);
        }
        else {
            intakeBelt.beltMotor.set(0.85);
        }
        
    }
    public void end(){
        intakeBelt.beltMotor.set(0);
    }
}


