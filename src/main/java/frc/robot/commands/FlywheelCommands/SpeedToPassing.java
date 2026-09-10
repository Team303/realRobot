package frc.robot.commands.FlywheelCommands;

import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Flywheel;
import static frc.robot.subsystems.Flywheel.FLYWHEEL_INTERP_GOAL;
import static frc.robot.RobotContainer.drive;

public class SpeedToPassing extends Command {

  private double goal_speed;
  private boolean leftSide;
  private Flywheel flywheel;

  public SpeedToPassing(Flywheel flywheel, boolean leftSide) {
    this.goal_speed = 0;
    this.flywheel = flywheel;
    this.leftSide = leftSide;
    addRequirements(flywheel);
  }

  @Override
  public void initialize() {
    goal_speed = drive.calculateFlyWheelSpeedPassing(leftSide);
 // goal_speed -= 2;
  }

  @Override
  public void execute() {
    //RobotContainer.shooting = true; 
    goal_speed = drive.calculateFlyWheelSpeedPassing(leftSide);
    flywheel.flywheelInterpNumber.set(goal_speed);
    // goal_speed = -41.5;
    //goal_speed = FLYWHEEL_INTERP_GOAL.getAsDouble();
    //System.out.println("goal_speed: " + goal_speed);
     flywheel.getToSpeed(goal_speed); //2520 - 6
    //flywheel.rightFlywheelMotor.setVoltage(6);
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