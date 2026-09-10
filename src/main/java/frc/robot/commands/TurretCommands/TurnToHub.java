package frc.robot.commands.TurretCommands;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;
import static frc.robot.RobotContainer.drive;


public class TurnToHub extends Command {
  private double goal;
  Turret turret;

  public TurnToHub(Turret turret) {
    addRequirements(turret);
    this.turret = turret;
  }

  @Override
  public void initialize() {
    //turret.createNewConfig();
    this.goal = turret.getTurretTurnPos() / 360.0;
    //System.out.println("GOAL: " + goal);
  }

  @Override
  public void execute() {
    //System.out.println("GOALLLLLL: " + turret.getTurretTurnPos());
    // goal = turret.getTurretTurnPos() / 360.0;

    // double tof = drive.getTOF(drive.getDistance());
    // double perpX = -Math.sin(goal);
    // double perpY = Math.cos(goal);

    // double vLateral = drive.getVelocity().getX() * perpX + drive.getVelocity().getY() * perpY;

    // double thetaOffset =  Math.atan2(vLateral * (0.03 + tof), drive.getDistance());
    
    // goal = goal + (thetaOffset * 0.18);


   // goal = (turret.getTurretTurnPos() / 360.0); // drive.whoKnows().getRotations() + 0.06591796875;
    goal = turret.getTurretTurnPos(drive.whoKnows()) / 360.0;
    System.out.println("helloooo: " + drive.whoKnows());
   // System.out.println("Goal: " + drive.whoKnows().getRotations());

    //double goal1=goal-(turret.getVelocity()*TimeOfFlight);
   // System.out.println("Rot goal: " + goal + " | Angle Goal: " + -turret.getTurretTurnPos());
    turret.moveToPos(goal);
    System.out.println("GOAL: " + goal + "; END: " + turret.getMotorPosition() + "; DIFF deg" + Math.abs(goal - turret.getMotorPosition()) * 360);
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