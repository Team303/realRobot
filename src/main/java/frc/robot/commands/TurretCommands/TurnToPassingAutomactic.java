package frc.robot.commands.TurretCommands;


import static frc.robot.RobotContainer.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.drive.Drive;

public class TurnToPassingAutomactic  extends Command {
  private final double GOAL_THRESHOLD = 0.0 / 360.0;
  private double goal; //Rotations
  private boolean leftSide;
  Turret turret;
  Drive drive;

  public TurnToPassingAutomactic(Turret turret) {
    addRequirements(turret);
    this.turret = turret;
  }

  @Override
  public void initialize() {
    int son = turret.getPassingSide();
    if (son == 1) {
      this.goal = turret.getTurretPassingPos(true) / 360.0;
    } else if (son == 0) {
      this.goal = turret.getTurretPassingPos(false) / 360.0;
    }
  }

  @Override
  public void execute() {
    //System.out.println("GOALLLLLL: " + turret.getTurretPassingPos(leftSide));
    int son = turret.getPassingSide();
    if (son == 1) {
      goal = turret.getTurretPassingPos(true) / 360.0;
      turret.moveToPos(goal);
    } else if (son == 0) {
      goal = turret.getTurretPassingPos(false) / 360.0;
      turret.moveToPos(goal);
    } else if (son == -1){
      //Home if its in the middle
      turret.moveToPos(0.23);
    }
   // System.out.println("Rot goal: " + goal + " | Angle Goal: " + -turret.getTurretTurnPos());
    //turret.moveToPos(goal);
    //System.out.println("GOAL: " + goal + "; END: " + turret.getMotorPosition() + "; DIFF" + Math.abs(goal - turret.getMotorPosition() * 360));
  }

  @Override
  public boolean isFinished() {
    return false;//Math.abs(turret.getMotorPosition()) > Constants.Shooter.Turret.HARD_MAX_TURRET_ROTATION;
    //return false;//Math.abs(goal - turret.getMotorPosition()) < GOAL_THRESHOLD;
  }

  @Override
  public void end(boolean interrupted) {
    //System.out.println("GOAL: " + goal + "; END: " + turret.getMotorPosition() + "; DIFF" + Math.abs(goal - turret.getMotorPosition()));
    turret.stopMotor();
  }
}