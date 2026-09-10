// package frc.robot.commands.ClimberCommands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.RobotContainer;
// import frc.robot.subsystems.Climber;

// public class GoToPosition extends Command {
//     double position;
//     Climber climber;
//     public GoToPosition(Climber climber, double position) {
//         this.climber = climber;
//         addRequirements(climber);
//         this.position = position;
//     }

//     @Override
//     public void initialize() {

//     }

//     @Override 
//     public void execute() {
//         climber.moveToSetpoint(10);
//         climber.position = 10;
//     }

//     @Override
//     public boolean isFinished() {
//         // return Math.abs((double) climber.getRealPosition(climber.climbMotor)
//         //                     - (double) climber.position)
//         //             < 0.05;

//         return false;
//     }

//     @Override
//     public void end(boolean interrupted) {
//       //  climber.climbMotor.set(0);
//     }
// }
