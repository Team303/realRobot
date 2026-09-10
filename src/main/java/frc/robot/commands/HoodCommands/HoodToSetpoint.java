package frc.robot.commands.HoodCommands;

import static frc.robot.subsystems.Hood.HOOD_GOAL_POS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.drive.Drive;
import static frc.robot.RobotContainer.drive;
import static frc.robot.subsystems.Hood.HOOD_INTERP_POS;

public class HoodToSetpoint extends Command {
  //private final double GOAL_THRESHOLD = 0 / 360.0;
  private double goal;
  private Hood hood;
 // private Drive drive;

  public HoodToSetpoint(Hood hood, double goal) {
    this.goal = goal;
  //  this.drive = drive;
    this.hood = hood;
        addRequirements(hood);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
  //System.out.println("hood goal: " + goal);
  hood.hoodFoundPos.set(goal);
  goal = HOOD_INTERP_POS.getAsDouble();
  // goal = 0.485;
    System.out.println("hood goal: " + goal);
    hood.moveToPos(goal);


    //hood.hoodMotor.setVoltage(-0.9);
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