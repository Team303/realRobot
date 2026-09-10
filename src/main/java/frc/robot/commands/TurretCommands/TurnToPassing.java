package frc.robot.commands.TurretCommands;


import static frc.robot.RobotContainer.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class TurnToPassing extends Command {
  private final double GOAL_THRESHOLD = 0.0 / 360.0;
  private double goal; //Rotations
  private boolean leftSide;
  Turret turret;

  public TurnToPassing(Turret turret, boolean leftSide) {
    addRequirements(turret);
    this.turret = turret;
    this.leftSide = leftSide;
  }

  @Override
  public void initialize() {
    //turret.createNewConfig();
    this.goal = turret.getTurretPassingPos(leftSide) / 360.0;
    //System.out.println("GOAL: " + goal);
  }

  @Override
  public void execute() {
    //System.out.println("GOALLLLLL: " + turret.getTurretPassingPos(leftSide));
    goal = turret.getTurretPassingPos(leftSide) / 360.0;

   // System.out.println("Rot goal: " + goal + " | Angle Goal: " + -turret.getTurretTurnPos());
    turret.moveToPos(goal);
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