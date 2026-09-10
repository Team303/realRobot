package frc.robot.commands.HoodCommands;

import static frc.robot.subsystems.Hood.HOOD_GOAL_POS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.drive.Drive;
import static frc.robot.RobotContainer.drive;
import static frc.robot.subsystems.Hood.HOOD_INTERP_POS;

public class RotateOnPassingAuto extends Command {
  //private final double GOAL_THRESHOLD = 0 / 360.0;
  private double goal;
  private Hood hood;
 // private Drive drive;

  public RotateOnPassingAuto(Hood hood) {
    this.goal = 0;
  //  this.drive = drive;
    this.hood = hood;
    addRequirements(hood);
  }

  @Override
  public void initialize() {
    int son = drive.getPassingSide();
    if (son == 1) {
      goal = drive.calculateHoodAnglePassing(true);
    } else if (son == 0) {
      goal = drive.calculateHoodAnglePassing(false);
    }
  }

  @Override
  public void execute() {
    int son = drive.getPassingSide();
    if (son == 1) {
      goal = drive.calculateHoodAnglePassing(true);
      hood.moveToPos(goal);
    } else if (son == 0) {
      goal = drive.calculateHoodAnglePassing(false);
      hood.moveToPos(goal);
    } else if (son == -1){
      //Home if its in the middle
      hood.moveToPos(0);
    }
  }


   @Override
  public boolean isFinished() {
    return false;//Math.abs(Constants.Shooter.Turret.TURRET_HOME_POS - hood.getMotorPosition()) < GOAL_THRESHOLD;
  }

  @Override
  public void end(boolean interrupted) {
    //System.out.println("GOAL: " + Constants.Shooter.Turret.TURRET_HOME_POS + "; END: " + hood.getMotorPosition() + "; DIFF" + Math.abs(Constants.Shooter.Turret.TURRET_HOME_POS - hood.getMotorPosition()));
    hood.hoodMotor.set(0);
  }

  // @Override
  // public boolean isFinished() {
  //   //System.out.println("isFinished Diff: " + Math.abs(goal - turret.throughBore.getPosition().getValueAsDouble()) + "; Thres: " + GOAL_THRESHOLD);
  //  return Math.abs(goal - hood.getMotorPosition()) < GOAL_THRESHOLD;
  // }

  // @Override
  // public void end(boolean interrupted) {
  //   System.out.println("GOAL: " + goal + "; END: " + hood.getMotorPosition() + "; DIFF" + Math.abs(goal - hood.getMotorPosition()));
  //   hood.stopMotors();
  // }
}