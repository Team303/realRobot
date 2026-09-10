package frc.robot.commands.SpindexerCommands;

import java.util.logging.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Spindexer;

import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.drive.Drive.dist;

public class spin2 extends Command{
    
    boolean forward;
    Spindexer spindexer;
    boolean fast;
    Timer timer;

    public spin2(Spindexer spindexer, boolean forward, boolean fast) {
        this.forward = forward;
        this.fast = fast;
        this.spindexer = spindexer;
        timer = new Timer();
        addRequirements(spindexer);
    }

    public void initialize() {
        // timer.reset();
        if (forward) {
            spindexer.spindexerMotor.set(fast ? 1 : 1);
        } else {
            spindexer.spindexerMotor.set(fast ? -1 : -1);
        }
        /*timer.start();
        while(timer.hasElapsed(Seconds.of(0.2))) {

        }
        timer.stop();
        timer.reset();
        var configs = spindexer.spindexerMotor.getConfigurator();
        CurrentLimitsConfigs clcs = new CurrentLimitsConfigs();

        clcs.StatorCurrentLimit = 40;
        clcs.SupplyCurrentLimit = 40;
    
        configs.apply(clcs);*/

        
    }

    public void execute() {
        //System.out.println("spinning");
        
        
    }

    public void end() {
        spindexer.spindexerMotor.set(0);
        var configs = spindexer.spindexerMotor.getConfigurator();
        CurrentLimitsConfigs clcs = new CurrentLimitsConfigs();

        clcs.StatorCurrentLimit = 120;
        clcs.SupplyCurrentLimit = 120;
    
        configs.apply(clcs);

    }
}
