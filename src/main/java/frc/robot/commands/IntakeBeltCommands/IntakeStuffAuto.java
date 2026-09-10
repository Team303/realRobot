package frc.robot.commands.IntakeBeltCommands;

import static frc.robot.RobotContainer.flywheel;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeBelt;

public class IntakeStuffAuto extends Command {
    
    IntakeBelt intakeBelt;

    public IntakeStuffAuto(IntakeBelt intakebelt) {
        addRequirements(intakebelt);
        this.intakeBelt = intakebelt;
        
    }
    public void initialize(){
        
    }
    public void execute(){

        if(RobotContainer.shooting) {
              var configs = intakeBelt.beltMotor.getConfigurator();
        CurrentLimitsConfigs clcs = new CurrentLimitsConfigs();

        clcs.StatorCurrentLimit = 60;
        clcs.SupplyCurrentLimit = 60;
    
        configs.apply(clcs);
        
        } else {
             var configs = intakeBelt.beltMotor.getConfigurator();
        CurrentLimitsConfigs clcs = new CurrentLimitsConfigs();

        clcs.StatorCurrentLimit = 80;
        clcs.SupplyCurrentLimit = 80;
    
        configs.apply(clcs);
        }
        intakeBelt.beltMotor.set(-1);
        intakeBelt.takeMotor.set(DriverStation.isAutonomous() ? -0.1 : 0);
        
    }
    public void end(){
        intakeBelt.beltMotor.set(0);
        intakeBelt.takeMotor.set(0);
    }
}


