// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.InterpolatingMatrixTreeMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.Turret;
//import frc.robot.subsystems.LED;
import frc.robot.Constants.LED.AnimationType;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveToPoseML;
import frc.robot.commands.DriveToPoseStraight;
//import frc.robot.commands.ClimberCommands.ClimbDefault;
//import frc.robot.commands.ClimberCommands.GoToPosition;
import frc.robot.commands.FlywheelCommands.DefaultFlywheel;
import frc.robot.commands.FlywheelCommands.SpeedToPassing;
import frc.robot.commands.FlywheelCommands.SpeedToPassingAuto;
import frc.robot.commands.FlywheelCommands.SpeedToSetpoint;
import frc.robot.commands.FlywheelCommands.TurnToSpeed;
import frc.robot.commands.HoodCommands.HoodDefault;
import frc.robot.commands.HoodCommands.HoodToSetpoint;
import frc.robot.commands.HoodCommands.RotateOnPassing;
import frc.robot.commands.HoodCommands.RotateOnPassingAuto;
import frc.robot.commands.HoodCommands.RotateToPosition;
import frc.robot.commands.IntakeBeltCommands.IntakeDefault;
import frc.robot.commands.IntakeBeltCommands.IntakeDown;
import frc.robot.commands.IntakeBeltCommands.IntakeStuff;
import frc.robot.commands.IntakeBeltCommands.IntakeStuffAuto;
import frc.robot.commands.IntakeBeltCommands.IntakeStuffRemake;
import frc.robot.commands.IntakeBeltCommands.IntakeUp;
//import frc.robot.commands.LEDCommands.SetLEDs;
import frc.robot.commands.SpindexerCommands.SpinDefault;
import frc.robot.commands.SpindexerCommands.spin2;
import frc.robot.commands.TurretCommands.HomeTurret;
import frc.robot.commands.TurretCommands.TurnToHub;
import frc.robot.commands.TurretCommands.TurnToHubAuto;
import frc.robot.commands.TurretCommands.TurnToPassing;
import frc.robot.commands.TurretCommands.TurnToPassingAutomactic;
import frc.robot.commands.TurretCommands.TurnToPosition;
import frc.robot.generated.TunerConstants;
//import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.IntakeBelt;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public static Drive drive;

  public static Vision vision;

    public static Spindexer spindexer;

    //public static Climber climber;

    public static Flywheel flywheel;

    public static Hood hood;

    public static IntakeBelt intakeBelt;

    public static Turret turret;

    //public static LED led;

    public static boolean shooting;


  //  public static IntakeBelt intakeBelt;
  // Controller
  public static CommandXboxController controller = new CommandXboxController(0);
  public static CommandXboxController opController = new CommandXboxController(1);
  public static Joystick flightStick = new Joystick(2);


  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    shooting = false;
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
               // new VisionIOLimelight("limelight-rturret", drive::getRotation),
                new VisionIOLimelight("limelight-lturret", drive::getRotation),
                new VisionIOLimelight("limelight-rturret", drive::getRotation),
                new VisionIOLimelight("limelight-front", drive::getRotation));
        spindexer = new Spindexer();
        flywheel = new Flywheel();
        //climber = null;//new Climber();
        hood = new Hood();
        intakeBelt = new IntakeBelt();
        turret = new Turret();
        //led = new LED();
       // intakeBelt = new IntakeBelt();
        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    configureNamedCommands();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
    // NamedCommands.register

  }

  public void configureNamedCommands() {
    // NamedCommands.registerCommand("Climb L1", new GoToPosition(climber, 10));
    // NamedCommands.registerCommand("Lower Climb", new GoToPosition(climber, 0));
    NamedCommands.registerCommand("simple", new TurnToSpeed(flywheel));
    //NamedCommands.registerCommand("Stop", new ParallelCommandGroup(new DefaultFlywheel(flywheel), new HoodDefault(hood), new SpinDefault(spindexer)));
    NamedCommands.registerCommand("just turret", new TurnToHubAuto(turret));
    NamedCommands.registerCommand("Shoot1", new ParallelCommandGroup(new TurnToSpeed(flywheel), new RotateToPosition(hood)));
    NamedCommands.registerCommand("Shoot2", new TurnToHubAuto(turret));
    NamedCommands.registerCommand("Shoot3", new spin2(spindexer, true, true));
    NamedCommands.registerCommand("just hood", new RotateToPosition(hood));
    NamedCommands.registerCommand("intake", new IntakeStuffAuto(intakeBelt));
    NamedCommands.registerCommand("Stop Intake", new IntakeDefault(intakeBelt));
    NamedCommands.registerCommand("intake up", new IntakeUp(intakeBelt));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
    private void configureButtonBindings() {
        intakeBelt.setDefaultCommand(new IntakeDefault(intakeBelt));
        spindexer.setDefaultCommand(new SpinDefault(spindexer));
        hood.setDefaultCommand(new HoodDefault(hood));
        flywheel.setDefaultCommand(new DefaultFlywheel(flywheel));
      //  led.setDefaultCommand(new SetLEDs(led, AnimationType.Solid, AnimationType.ColorFlow));

        //opController.back().toggleOnTrue(new SetLEDs(led, AnimationType.Rainbow, AnimationType.SingleFade));

        opController.y().toggleOnTrue(new ParallelCommandGroup(new TurnToHub(turret), new RotateToPosition(hood), new TurnToSpeed(flywheel)));
        opController.x().toggleOnTrue(new ParallelCommandGroup(new TurnToPassing(turret, true), new RotateOnPassing(hood, true), new SpeedToPassing(flywheel, true)));
        opController.b().toggleOnTrue(new ParallelCommandGroup(new TurnToPassing(turret, false), new RotateOnPassing(hood, false), new SpeedToPassing(flywheel, false)));
        opController.a().toggleOnTrue(new ParallelCommandGroup(new TurnToPassingAutomactic(turret), new RotateOnPassingAuto(hood), new SpeedToPassingAuto(flywheel)));

        opController.leftTrigger().toggleOnTrue(new spin2(spindexer, false, true));
        opController.rightTrigger().toggleOnTrue(new spin2(spindexer, true, false));

        opController.leftBumper().toggleOnTrue(new IntakeStuff(intakeBelt, false));
        opController.rightBumper().toggleOnTrue(new IntakeStuffAuto(intakeBelt));


        opController.povDown().toggleOnTrue(new ParallelCommandGroup(new TurnToPosition(turret, 0.37255859375),new SpeedToSetpoint(flywheel, flywheel.flywheelInterpNumber.getAsDouble()),  new HoodToSetpoint(hood, hood.HOOD_INTERP_POS.getAsDouble())));
        //opController.povLeft().toggleOnTrue(new TurnToPosition(turret, 0.37255859375));
        //opController.povUp().toggleOnTrue(new SpeedToSetpoint(flywheel, flywheel.flywheelInterpNumber.getAsDouble()));
        //opController.povRight().toggleOnTrue(new HoodToSetpoint(hood, hood.HOOD_INTERP_POS.getAsDouble()));
    //    opController.pov(90).toggleOnTrue(new IntakeUp(intakeBelt));        

        //NEED TO COMPLETE SETPOINTS
     //   opController.pov(0).toggleOnTrue(new ParallelCommandGroup(new TurnToHub(turret), new HoodToSetpoint(hood, 0.12639097555576206), new SpeedToSetpoint(flywheel, -40.45454095621027)));     
    //    opController.pov(180).toggleOnTrue(new IntakeDown(intakeBelt));

        //TESTING ONLY COMMANDS
        /* 
        //Left Long
        opController.povLeft().toggleOnTrue(new ParallelCommandGroup(new TurnToPassing(turret, true), new HoodToSetpoint(hood, 1.0), new SpeedToSetpoint(flywheel, -79.5)));
        //Left short
        opController.povUp().toggleOnTrue(new ParallelCommandGroup(new TurnToPassing(turret, true), new HoodToSetpoint(hood, 1.15), new SpeedToSetpoint(flywheel, -65.5)));

        //right long
        opController.povRight().toggleOnTrue(new ParallelCommandGroup(new TurnToPassing(turret, false), new HoodToSetpoint(hood, 1.0), new SpeedToSetpoint(flywheel, -79.5)));

        //right short
        opController.povDown().toggleOnTrue(new ParallelCommandGroup(new TurnToPassing(turret, false), new HoodToSetpoint(hood, 1.15), new SpeedToSetpoint(flywheel, -65.5)));
        */
        //opController.povLeft().toggleOnTrue(new ParallelCommandGroup(new TurnToPosition(turret, 0.03), new HoodToSetpoint(hood, 1.6), new SpeedToSetpoint(flywheel, -20.0))); //TESTING
     //   opController.pov(270).toggleOnTrue(new ParallelCommandGroup(new TurnToPosition(turret, 0.40), new HoodToSetpoint(hood, 0.7), new SpeedToSetpoint(flywheel, -45.0))); //TESTING        
        //opController.povLeft().toggleOnTrue(new IntakeStuffRemake(intakeBelt, true));
        //opController.povUp().toggleOnTrue(new IntakeUp(intakeBelt));
//     //opController.back().toggleOnTrue(new RotateToPosition(hood, 0.3));
//     opController.leftStick().toggleOnTrue(new spin2(spindexer, false, true));
//     opController.pov(0).toggleOnTrue(new IntakeStuff(intakeBelt, true));
//     opController.pov(180).toggleOnTrue(new IntakeStuff(intakeBelt, false));
//     opController.pov(270).toggleOnTrue(new IntakeDown(intakeBelt));
//     //opController.pov(90).toggleOnTrue(new TurnToSpeed(flywheel, -44));

//     //ACTUAL FLYWHEEL/HOOD COMMANDS
//     opController.leftBumper().toggleOnTrue(new RotateToPosition(hood));
//     opController.rightBumper().toggleOnTrue(new TurnToSpeed(flywheel));

//     opController.b().toggleOnTrue(new ParallelCommandGroup(new TurnToHub(turret), new RotateToPosition(hood), new TurnToSpeed(flywheel)));
   
//     //TESTING FLYWHEEL/HOOD COMMANDS
//     //opController.leftBumper().toggleOnTrue(new RotateToPosition(drive, hood));
//     //opController.rightBumper().toggleOnTrue(new TurnToSpeed(flywheel, -39.75));    
//     opController.rightStick().toggleOnTrue(new spin2(spindexer, true, true));
//     opController.rightTrigger().toggleOnTrue(new spin2(spindexer, true, false));
   
   
//     // opController.pov(270).toggleOnTrue(new RotateToPosition(hood, 0.05));

//    //opController.rightStick().toggleOnTrue(new GoToPosition(climber, 0.5));

//     //
//     //opController.a().toggleOnTrue(new HomeTurret(turret));
//     opController.a().toggleOnTrue(new ParallelCommandGroup(new TurnToPassing(turret, false), new RotateOnPassing(hood), new SpeedToPassing(flywheel)));
//     opController.x().toggleOnTrue(new TurnToPassing(turret, true));
//     //opController.a().toggleOnTrue(new TurnToPassing(turret, false));
//     opController.y().toggleOnTrue(new TurnToHub(turret));

//     opController.start().toggleOnTrue(new IntakeStuffAuto(intakeBelt));


    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY() ,
            () -> -controller.getLeftX() ,
            () -> -controller.getRightX()));

    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () -> -flightStick.getY() ,
    //         () -> -flightStick.getX() ,
    //         () -> -flightStick.getThrottle()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller
        .povLeft()
        .toggleOnTrue(
            new ParallelCommandGroup(
                new DriveToPoseStraight(
                    drive, new Pose2d(
                        new Translation2d(
                            DriverStation.getAlliance().get() == Alliance.Blue ? 2.815 : 16.54 - 2.185, 5.611), 
                        new Rotation2d(Units.degreesToRadians(
                            DriverStation.getAlliance().get() == Alliance.Blue ? -40 : 40))))
                //new SetLEDs(led, AnimationType.Strobe, AnimationType.Strobe
                ));


    controller
        .povUp()
        .toggleOnTrue(
            new ParallelCommandGroup(
                new DriveToPoseStraight(
                    drive, new Pose2d(new Translation2d(DriverStation.getAlliance().get() == Alliance.Blue ? 2.266 : 16.54 - 2.266, 3.847), new Rotation2d(DriverStation.getAlliance().get() == Alliance.Blue ? Units.degreesToRadians(0) : Units.degreesToRadians(180))))
                //new SetLEDs(led, AnimationType.Strobe, AnimationType.Strobe)
                ));

    controller
        .povRight()
        .toggleOnTrue(
            new ParallelCommandGroup(
                new DriveToPoseStraight(
                    drive, new Pose2d(new Translation2d(DriverStation.getAlliance().get() == Alliance.Blue ? 2.966 : 16.54 - 2.966, 2.168), new Rotation2d(DriverStation.getAlliance().get() == Alliance.Blue ? Units.degreesToRadians(50) : Units.degreesToRadians(50))))
            //new SetLEDs(led, AnimationType.Strobe, AnimationType.Strobe)
            ));

                

    // opController
    //     .start()
    //     .toggleOnTrue(
    //         new DriveToPoseStraight(
    //             drive, new Pose2d(new Translation2d(1.08, 4.6), new Rotation2d(Units.degreesToRadians(0)))));

    //opController.a().toggleOnTrue(new DriveToPoseML(drive));
    // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
    //                 drive)
    //             .ignoringDisable(true));

   // controller.leftBumper().toggleOnTrue(new GoToPosition(climber, 10));
   // controller.leftBumper().toggleOnFalse(new GoToPosition(climber, 0));

   // controller.y().toggleOnTrue()
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
   return autoChooser.get();
 //  return 
  }
}


// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 
// I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN I AM FADING ALEX MASIN 