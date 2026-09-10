package frc.robot.commands.FlywheelCommands;

import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Flywheel;
import static frc.robot.subsystems.Flywheel.FLYWHEEL_INTERP_GOAL;
import static frc.robot.RobotContainer.drive;

public class SpeedToSetpoint extends Command {

  private double goal_speed;
  private Flywheel flywheel;

  public SpeedToSetpoint(Flywheel flywheel, double goal_speed) {
    this.goal_speed = goal_speed;
    this.flywheel = flywheel;
    addRequirements(flywheel);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    RobotContainer.shooting = false; 
    flywheel.flywheelInterpNumber.set(goal_speed);
    // goal_speed = -41.5;
    goal_speed = FLYWHEEL_INTERP_GOAL.getAsDouble();
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
    //System.out.println("GOAL: " + Constants.Shooter.Turret.TURRET_HOME_POS + "; END: " + hood.getMotorPosition() + "; DIFF" + Math.abs(Constants.Shooter.Turret.TURRET_HOME_POS - hood.getMotorPosition()));
    flywheel.stopMotors();
    RobotContainer.shooting = false;
  }
}