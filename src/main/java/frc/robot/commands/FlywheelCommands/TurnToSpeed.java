package frc.robot.commands.FlywheelCommands;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Flywheel;
import static frc.robot.subsystems.Flywheel.FLYWHEEL_INTERP_GOAL;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.RobotContainer.drive;
import static frc.robot.RobotContainer.spindexer;

public class TurnToSpeed extends Command {

  private double goal_speed;
  private Flywheel flywheel;
  Timer timer;

  public TurnToSpeed(Flywheel flywheel) {
    this.goal_speed = 0;
    this.flywheel = flywheel;
    timer = new Timer();
    addRequirements(flywheel);
  }

  @Override
  public void initialize() {
    goal_speed = drive.calculateFlyWheelSpeed();
    goal_speed -= 1;

        RobotContainer.shooting = true; 
    goal_speed = drive.calculateFlyWheelSpeed(drive.whoKnows());
    flywheel.flywheelInterpNumber.set(goal_speed);


     timer.reset();
      timer.start();
        while(timer.hasElapsed(Seconds.of(0.2))) {

        }
        timer.stop();
        timer.reset();
        var configs = flywheel.leftFlywheelMotor.getConfigurator();
        CurrentLimitsConfigs clcs = new CurrentLimitsConfigs();

        clcs.StatorCurrentLimit = 60;
        clcs.SupplyCurrentLimit = 60;
    
        configs.apply(clcs);

        configs = flywheel.rightFlywheelMotor.getConfigurator();
        clcs = new CurrentLimitsConfigs();

        clcs.StatorCurrentLimit = 60;
        clcs.SupplyCurrentLimit = 60;
    
        configs.apply(clcs);

        configs = flywheel.kickerMotor.getConfigurator();
        clcs = new CurrentLimitsConfigs();

        clcs.StatorCurrentLimit = 80;
        clcs.SupplyCurrentLimit = 80;
    
        configs.apply(clcs);
  }

  @Override
  public void execute() {
    RobotContainer.shooting = true; 
    goal_speed = drive.calculateFlyWheelSpeed(drive.whoKnows());
    flywheel.flywheelInterpNumber.set(goal_speed);
    // goal_speed = -41.5;
    //goal_speed = FLYWHEEL_INTERP_GOAL.getAsDouble();
    //System.out.println("goal_speed: " + goal_speed);
     flywheel.getToSpeed(goal_speed); //2520 - 6
    //flywheel.rightFlywheelMotor.setVoltage(6);

    // flywheel.leftFlywheelMotor.set(1);
    // flywheel.rightFlywheelMotor.set(-1);
    
    //System.out.println("left v: " + flywheel.getLeftMotorSpeed());
    //System.out.println("right v: " + flywheel.getRightMotorSpeed());


    // System.out.println("left a: " + flywheel.getLeftMotorAccel());
    // System.out.println("right a: " + flywheel.getRightMotorAccel());
    //System.out.println("kicker speed: " + flywheel.getKickerMotorSpeed());

    //hood.moveToPos(Constants.Shooter.Turret.TURRET_HOME_POS);

    // flywheel.leftFlywheelMotor.set(0.5);
  }

  @Override
  public boolean isFinished() {
    return false;//Math.abs(Constants.Shooter.Turret.TURRET_HOME_POS - hood.getMotorPosition()) < GOAL_THRESHOLD;
  }

  @Override
  public void end(boolean interrupted) {

     var configs = flywheel.leftFlywheelMotor.getConfigurator();
        CurrentLimitsConfigs clcs = new CurrentLimitsConfigs();

        clcs.StatorCurrentLimit = 120;
        clcs.SupplyCurrentLimit = 120;
    
        configs.apply(clcs);


        configs = flywheel.rightFlywheelMotor.getConfigurator();
        clcs = new CurrentLimitsConfigs();

        clcs.StatorCurrentLimit = 120;
        clcs.SupplyCurrentLimit = 120;
    
        configs.apply(clcs);


        configs = flywheel.kickerMotor.getConfigurator();
        clcs = new CurrentLimitsConfigs();

        clcs.StatorCurrentLimit = 120;
        clcs.SupplyCurrentLimit = 120;
    
        configs.apply(clcs);
    //System.out.println("GOAL: " + Constants.Shooter.Turret.TURRET_HOME_POS + "; END: " + hood.getMotorPosition() + "; DIFF" + Math.abs(Constants.Shooter.Turret.TURRET_HOME_POS - hood.getMotorPosition()));
    flywheel.stopMotors();
    RobotContainer.shooting = false;
  }
}