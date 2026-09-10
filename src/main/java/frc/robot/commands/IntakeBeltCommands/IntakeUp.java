package frc.robot.commands.IntakeBeltCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeBelt;


public class IntakeUp extends Command{
    
    IntakeBelt take;

    public IntakeUp(IntakeBelt take) {
        this.take = take;
        addRequirements(take);
    }

   public void execute(){
        take.takeMotor.set(0.25);
        take.beltMotor.set(-0.5);
    }
}
