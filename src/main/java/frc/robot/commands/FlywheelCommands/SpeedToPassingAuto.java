package frc.robot.commands.FlywheelCommands;

import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Flywheel;
import static frc.robot.subsystems.Flywheel.FLYWHEEL_INTERP_GOAL;
import static frc.robot.RobotContainer.drive;

public class SpeedToPassingAuto extends Command {

  private double goal_speed;
  private Flywheel flywheel;

  public SpeedToPassingAuto(Flywheel flywheel) {
    this.goal_speed = 0;
    this.flywheel = flywheel;
    addRequirements(flywheel);
  }

  @Override
  public void initialize() {
    int son = drive.getPassingSide();
    if (son == 1) {
      goal_speed = drive.calculateFlyWheelSpeedPassing(true);
    } else if (son == 0) {
      goal_speed = drive.calculateFlyWheelSpeedPassing(false);
    }
  }

  @Override
  public void execute() {
    //RobotContainer.shooting = true; 
    int son = drive.getPassingSide();
    if (son == 1) {
      goal_speed = drive.calculateFlyWheelSpeedPassing(true);
      flywheel.getToSpeed(goal_speed);
    } else if (son == 0) {
      goal_speed = drive.calculateFlyWheelSpeedPassing(false);
      flywheel.getToSpeed(goal_speed);
    } else if (son == -1) {
      //Home if its in the middle
      flywheel.getToSpeed(0);
    }
    flywheel.flywheelInterpNumber.set(goal_speed);
    // goal_speed = -41.5;
    //goal_speed = FLYWHEEL_INTERP_GOAL.getAsDouble();
    //System.out.println("goal_speed: " + goal_speed);
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