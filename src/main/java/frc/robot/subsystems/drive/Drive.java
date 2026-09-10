// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotContainer.hood;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Mode;
import frc.robot.commands.TurretCommands.TurnToHub;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.LoggedTunableNumber;

import java.time.chrono.IsoChronology;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class Drive extends SubsystemBase {

  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;

  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleSubscriber taSubscriber;

  private static final double BLUE_HUB_X = 4.6269;
  private static final double BLUE_HUB_Y = 4.03;
  private static final double RED_HUB_X = 11.91358;
  private static final double RED_HUB_Y = 4.03;

  private static final double BLUE_PASSING_LEFT_X = 1.513;
  private static final double BLUE_PASSING_LEFT_Y = 6.500;
  private static final double BLUE_PASSING_RIGHT_X = 1.513;
  private static final double BLUE_PASSING_RIGHT_Y = 1.157;

  private static final double RED_PASSING_LEFT_X = 14.670;
  private static final double RED_PASSING_LEFT_Y = 1.429;
  private static final double RED_PASSING_RIGHT_X = 14.670;
  private static final double RED_PASSING_RIGHT_Y = 6.65;

  
  

  //   private final LimelightIOInputsAutoLogged inputs = new LimelightIOInputsAutoLogged();

  private final String limelightName;
  RobotConfig configs;

  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  public static LoggedTunableNumber xControllerP =
      new LoggedTunableNumber(
          "DriveToPoseStraight/xControllerP", Constants.DriveToPoseStraight.XController.kP);
  public static LoggedTunableNumber yControllerP =
      new LoggedTunableNumber(
          "DriveToPoseStraight/xControllerP", Constants.DriveToPoseStraight.YController.kP);
  public static LoggedTunableNumber ThetaConstrollerP =
      new LoggedTunableNumber(
          "DriveToPoseStraight/thetaConstrollerP",
          Constants.DriveToPoseStraight.ThetaController.kP);

  public static InterpolatingDoubleTreeMap hoodAngles;
  public static InterpolatingDoubleTreeMap flywheelSpeeds;
  public static InterpolatingDoubleTreeMap timeOfFlight;

  public static InterpolatingDoubleTreeMap hoodAnglesPASSING;
  public static InterpolatingDoubleTreeMap flywheelSpeedsPASSING;



  private static LoggedNetworkNumber distanceLogged;


  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = Units.lbsToKilograms(134);
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  public final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

        public static double dist;


  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.kZero;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {


        flywheelSpeeds = new InterpolatingDoubleTreeMap();
        hoodAngles = new InterpolatingDoubleTreeMap();
        timeOfFlight = new InterpolatingDoubleTreeMap();
        flywheelSpeedsPASSING = new InterpolatingDoubleTreeMap();
        hoodAnglesPASSING = new InterpolatingDoubleTreeMap();
        //Distance and Angle
        // hoodAngles.put(3.7338, 0.48);
        // hoodAngles.put(4.1148, 0.6 - 0.015);  
        // hoodAngles.put(2.4638, 0.2);  
        // hoodAngles.put(3.2766, 0.35);
        // hoodAngles.put(4.4196, 0.65 - 0.02);  
        // hoodAngles.put(4.9550, 1.0 - 0.05);
        // hoodAngles.put(1.6002, 0.0);
        // hoodAngles.put(2.3876, 0.1);

        // hoodAngles.put(Units.inchesToMeters(105), 0.3);
        // hoodAngles.put(Units.inchesToMeters(74.5), 0.04);
        // hoodAngles.put(Units.inchesToMeters(134), 0.38);
        // hoodAngles.put(Units.inchesToMeters(139), 0.39);
        // hoodAngles.put(Units.inchesToMeters(56), 0.00);
        // hoodAngles.put(Units.inchesToMeters(85), 0.294); //FINISH TUNING THIS ONE
        // hoodAngles.put(Units.inchesToMeters(124), 0.485);

        // hoodAngles.put(2.28, 0.07);
        // hoodAngles.put(2.557, 0.12);
        //hoodAngles.put(2.6, 0.2);
        //hoodAngles.put(2.72, 0.3);
        //hoodAngles.put(3.03, 0.35);
        //hoodAngles.put(3.255, 0.365);
        //hoodAngles.put(3.343, 0.43);
        //hoodAngles.put(4.05, 0.7);

      //   hoodAngles.put(1.762, 0.00);
      //   hoodAngles.put(2.0551861347569482, 0.043);
      //   hoodAngles.put(2.87, 0.27);
      //   hoodAngles.put(3.102, 0.35);
      //   hoodAngles.put(3.400, 0.35); 
      //   hoodAngles.put(3.842, 0.45);
      //   hoodAngles.put(4.120, 0.60);
      //   hoodAngles.put(4.790, 0.80);
      //  hoodAngles.put(5.244, 0.95);

        hoodAngles.put(1.377756573567455, -0.01);
        hoodAngles.put(1.658323639241114, 0.0);
        hoodAngles.put(2.1059877035368135, 0.05);
        hoodAngles.put(2.254178135902522, 0.0857);
        hoodAngles.put(2.4347267611588195, 0.13);
        hoodAngles.put(2.8783984137608805, 0.22);
        hoodAngles.put(3.171564531171188, 0.25);
        hoodAngles.put(3.380000000000000, 0.42);
        hoodAngles.put(3.6348421504160573, 0.45);
        hoodAngles.put(3.95287091546086, 0.49);
        hoodAngles.put(4.296466849582756, 0.53);
        hoodAngles.put(4.662327337938311, 0.80);
        hoodAngles.put(4.869053261867801, 0.90);



    
    //    hoodAngles.put(5.25, 0.0);
   



        //Distance and Speed

        // flywheelSpeeds.put(3.7338, -39.75 - 0.75);
        // flywheelSpeeds.put(4.1148, -41.5 - 1.5);  
        // flywheelSpeeds.put(2.4638, -34.0);  
        // flywheelSpeeds.put(3.2766, -37.0);
        // flywheelSpeeds.put(4.4196, -43.5 - 1.8);  
        // flywheelSpeeds.put(4.9550, -48.75 - 2);
        // flywheelSpeeds.put(1.6002, -33.0);
        // flywheelSpeeds.put(2.3876, -36.0);

        // flywheelSpeeds.put(Units.inchesToMeters(105), -41.5);
        // flywheelSpeeds.put(Units.inchesToMeters(74.5), -38.0);
        // flywheelSpeeds.put(Units.inchesToMeters(134), -43.0);
        // flywheelSpeeds.put(Units.inchesToMeters(139), -42.0);
        // flywheelSpeeds.put(Units.inchesToMeters(56), -36.0);
        // flywheelSpeeds.put(Units.inchesToMeters(85), -36.0); //FINISH TUNING THIS ONE
        // flywheelSpeeds.put(Units.inchesToMeters(124), -40.5);
        
                // flywheelSpeeds.put(2.254, -40.0);
        // flywheelSpeeds.put(2.557, -40.5);
        //flywheelSpeeds.put(2.6, -40.0);
        //flywheelSpeeds.put(2.72, -40.0);
        //flywheelSpeeds.put(3.03, -40.0);
        //flywheelSpeeds.put(3.255, -40.0);
        //flywheelSpeeds.put(3.343, -43.5);
        //flywheelSpeeds.put(4.05, -45.0);

        // flywheelSpeeds.put(1.762, -38.0);
        // flywheelSpeeds.put(2.0551861347569482, -39.5);
        // flywheelSpeeds.put(2.87, -42.7);
        // flywheelSpeeds.put(3.102, -42.72);
        // flywheelSpeeds.put(3.400, -42.75);
        // flywheelSpeeds.put(3.842, -44.75);
        // flywheelSpeeds.put(4.120, -45.5);
        // flywheelSpeeds.put(4.790, -47.5);
        // flywheelSpeeds.put(5.244, -48.25);

        flywheelSpeeds.put(1.377756573567455, -37.0);
        flywheelSpeeds.put(1.658323639241114, -37.75);
        flywheelSpeeds.put(2.1059877035368135, -38.5);
        flywheelSpeeds.put(2.254178135902522, -38.942);
        flywheelSpeeds.put(2.4347267611588195, -39.5);
        flywheelSpeeds.put(2.8783984137608805, -41.25);
        flywheelSpeeds.put(3.171564531171188, -42.25);
        flywheelSpeeds.put(3.380000000000000, -42.75);
        flywheelSpeeds.put(3.6348421504160573, -43.5);
        flywheelSpeeds.put(3.95287091546086, -44.5);
        flywheelSpeeds.put(4.296466849582756, -45.75);
        flywheelSpeeds.put(4.662327337938311, -46.50);
        flywheelSpeeds.put(4.869053261867801, -47.5);
      //  flywheelSpeeds.put(5.25, 0.0);
        
        
      //Theortical Passing ones
      //hoodAnglesPASSING.put(5.5, 1.05);
      hoodAnglesPASSING.put(6.0, 0.9);
      hoodAnglesPASSING.put(6.5, 0.9);
      hoodAnglesPASSING.put(7.0, 0.9);
      hoodAnglesPASSING.put(7.5, 0.9);
      hoodAnglesPASSING.put(8.0, 0.91);
      hoodAnglesPASSING.put(8.5, 0.92);
      hoodAnglesPASSING.put(9.0, 0.92);
      hoodAnglesPASSING.put(9.5, 0.92);
      hoodAnglesPASSING.put(10.0, 0.92);
      hoodAnglesPASSING.put(10.5,0.92);
      hoodAnglesPASSING.put(11.0, 0.9);//this and next two with full battery values
      hoodAnglesPASSING.put(11.5, 0.9);
      hoodAnglesPASSING.put(12.0, 0.9);
      //hoodAnglesPASSING.put(14.5, 1.0);
      
      //Theortical Passing ones
      //][\flywheelSpeedsPASSING.put(5.5, -50.5 - 1);
      flywheelSpeedsPASSING.put(6.0, -48.3);
      flywheelSpeedsPASSING.put(6.5, -49.7);
      flywheelSpeedsPASSING.put(7.0, -51.3);
      flywheelSpeedsPASSING.put(7.5, -54.67);
      flywheelSpeedsPASSING.put(8.0, -58.34);
      flywheelSpeedsPASSING.put(8.5, -60.12);
      flywheelSpeedsPASSING.put(9.0, -62.22);
      flywheelSpeedsPASSING.put(9.5, -72.22);
      flywheelSpeedsPASSING.put(10.0, -75.22);
      flywheelSpeedsPASSING.put(10.5, -75.22);
      flywheelSpeedsPASSING.put(11.0, -75.0);//this one and next two are with full battery values idk
      flywheelSpeedsPASSING.put(11.5, -77.0);
      flywheelSpeedsPASSING.put(12.0, -80.0);
      //flywheelSpeedsPASSING.put(14.5, -74.5);

      timeOfFlight.put(2.6017, 1.01);
      timeOfFlight.put(1.9418,0.94);
      timeOfFlight.put(4.4621,1.06);
      timeOfFlight.put(3.860, 1.12);
      timeOfFlight.put(3.13,1.07);


    distanceLogged = new LoggedNetworkNumber("Distance Logged", 0.0);

    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    this.limelightName = "limelight-machine";
    var table = NetworkTableInstance.getDefault().getTable(limelightName);
    System.out.println("Table: ");
    System.out.println(table);
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    taSubscriber = table.getDoubleTopic("ta").subscribe(0.0);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    try {
      configs = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(30.0, 0.0, 0.0), new PIDConstants(30.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[0]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {

    //System.out.println("Velocity: " + getVelocity());
    //System.out.println(getPose());
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  public Pose2d getFuelPose2dPath() {
    if (!fuelInVision()) {
      return null;
    }
    double xAngle = txSubscriber.get();
    // Rotation2d rotation =
    //         new Rotation2d(robotPoseSupplier.get().getRotation().getRadians() + (xAngle *
    // (Math.PI / 180)));

    Rotation2d rotation =
        new Rotation2d(getPose().getRotation().getRadians() + ((xAngle*0.8) * (Math.PI / 180)));

    return new Pose2d(
       new Translation2d(getPose().getTranslation().getX() /*meters*/, getPose().getTranslation().getY())
           .plus(new Translation2d(1.5, 0).rotateBy(Rotation2d.fromDegrees(rotation.getDegrees()))), rotation);
        //        ,
      //  rotation //);
//);
  }

  public double getDistance() {
          Pose2d currentPose = getPose(); 
      currentPose = currentPose.plus(new Transform2d(new Translation2d(Constants.Shooter.Turret.OFFSET_POS_X, Constants.Shooter.Turret.OFFSET_POS_Y).rotateBy(new Rotation2d(getPose().getRotation().getRadians())), new Rotation2d(0)));
          double distance = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Math.sqrt(Math.pow((currentPose.getX() - BLUE_HUB_X), 2) + Math.pow((currentPose.getY() - BLUE_HUB_Y), 2)) : Math.sqrt(Math.pow((currentPose.getX() - RED_HUB_X), 2) + Math.pow((currentPose.getY() - RED_HUB_Y), 2));
  return distance;
        }

      


  public double calculateFlyWheelSpeed() {
      Pose2d currentPose = getPose(); 
      currentPose = currentPose.plus(new Transform2d(new Translation2d(Constants.Shooter.Turret.OFFSET_POS_X, Constants.Shooter.Turret.OFFSET_POS_Y).rotateBy(new Rotation2d(getPose().getRotation().getRadians())), new Rotation2d(0)));
      double distance = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Math.sqrt(Math.pow((currentPose.getX() - BLUE_HUB_X), 2) + Math.pow((currentPose.getY() - BLUE_HUB_Y), 2)) : Math.sqrt(Math.pow((currentPose.getX() - RED_HUB_X), 2) + Math.pow((currentPose.getY() - RED_HUB_Y), 2));
      dist = distance;
      distanceLogged.set(distance);
      return flywheelSpeeds.get(distance);
    }


     public double calculateFlyWheelSpeed(Pose2d pose) {
   //   Pose2d currentPose = getPose(); 
   //   currentPose = currentPose.plus(new Transform2d(new Translation2d(Constants.Shooter.Turret.OFFSET_POS_X, Constants.Shooter.Turret.OFFSET_POS_Y).rotateBy(new Rotation2d(getPose().getRotation().getRadians())), new Rotation2d(0)));
      double distance = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Math.sqrt(Math.pow((pose.getX() - BLUE_HUB_X), 2) + Math.pow((pose.getY() - BLUE_HUB_Y), 2)) : Math.sqrt(Math.pow((pose.getX() - RED_HUB_X), 2) + Math.pow((pose.getY() - RED_HUB_Y), 2));
      dist = distance;
      distanceLogged.set(distance);
      return flywheelSpeeds.get(distance);
    }

   public Pose2d whoKnows() {
    // Step 1: Latency-compensated robot pose
    Pose2d estimatedPose = getPose().exp(
        new Twist2d(
            getChassisSpeeds().vxMetersPerSecond * 0.03,
            getChassisSpeeds().vyMetersPerSecond * 0.03,
            getChassisSpeeds().omegaRadiansPerSecond * 0.03
        )
    );

    // Step 2: Turret pose using proper transform
    Transform2d robotToTurret = new Transform2d(
        new Translation2d(
            Constants.Shooter.Turret.OFFSET_POS_X,
            Constants.Shooter.Turret.OFFSET_POS_Y
        ),
        new Rotation2d()
    );

    Pose2d turretPose = estimatedPose.plus(robotToTurret);

    // Step 3: Target position
    Translation2d target = new Translation2d(BLUE_HUB_X, BLUE_HUB_Y);

    double turretToTargetDistance = target.getDistance(turretPose.getTranslation());

    // Step 4: Turret velocity (field-relative)
    double robotAngle = estimatedPose.getRotation().getRadians();

    Translation2d robotToTurretTranslation = robotToTurret.getTranslation();

    double turretVelocityX =
        getChassisSpeeds().vxMetersPerSecond
            - getChassisSpeeds().omegaRadiansPerSecond
                * robotToTurretTranslation.getY();

    double turretVelocityY =
        getChassisSpeeds().vyMetersPerSecond
            + getChassisSpeeds().omegaRadiansPerSecond
                * robotToTurretTranslation.getX();

    // Rotate velocity into field frame
    double fieldVx =
        turretVelocityX * Math.cos(robotAngle)
            - turretVelocityY * Math.sin(robotAngle);

    double fieldVy =
        turretVelocityX * Math.sin(robotAngle)
            + turretVelocityY * Math.cos(robotAngle);

    // Step 5: Iterative lookahead
    Pose2d lookaheadPose = turretPose;
    double lookaheadDistance = turretToTargetDistance;

    for (int i = 0; i < 20; i++) {
        double timeOfFlightVal = timeOfFlight.get(lookaheadDistance);

        Translation2d offset = new Translation2d(
            fieldVx * timeOfFlightVal,
            fieldVy * timeOfFlightVal
        );

        lookaheadPose = new Pose2d(
            turretPose.getTranslation().plus(offset),
            turretPose.getRotation()
        );

        lookaheadDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput("LaunchCalculator/TurretToTargetDistance", lookaheadDistance);

    return lookaheadPose;
}
    

   

    public double calculateHoodAngle() {
      Pose2d currentPose = getPose(); 
      currentPose = currentPose.plus(new Transform2d(new Translation2d(Constants.Shooter.Turret.OFFSET_POS_X, Constants.Shooter.Turret.OFFSET_POS_Y).rotateBy(new Rotation2d(getPose().getRotation().getRadians())), new Rotation2d(0)));
      double distance = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Math.sqrt(Math.pow((currentPose.getX() - BLUE_HUB_X), 2) + Math.pow((currentPose.getY() - BLUE_HUB_Y), 2)) : Math.sqrt(Math.pow((currentPose.getX() - RED_HUB_X), 2) + Math.pow((currentPose.getY() - RED_HUB_Y), 2));
      distanceLogged.set(distance);
      return hoodAngles.get(distance);
    }

    public double calculateHoodAngle(Pose2d pose) {
 //     Pose2d currentPose = getPose(); 
   //   currentPose = currentPose.plus(new Transform2d(new Translation2d(Constants.Shooter.Turret.OFFSET_POS_X, Constants.Shooter.Turret.OFFSET_POS_Y).rotateBy(new Rotation2d(getPose().getRotation().getRadians())), new Rotation2d(0)));
      double distance = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Math.sqrt(Math.pow((pose.getX() - BLUE_HUB_X), 2) + Math.pow((pose.getY() - BLUE_HUB_Y), 2)) : Math.sqrt(Math.pow((pose.getX() - RED_HUB_X), 2) + Math.pow((pose.getY() - RED_HUB_Y), 2));
      distanceLogged.set(distance);
      return hoodAngles.get(distance);
    }

    public double calculateFlyWheelSpeedPassing(boolean leftSide) {
      Pose2d currentPose = getPose();
      currentPose = currentPose.plus(new Transform2d(new Translation2d(Constants.Shooter.Turret.OFFSET_POS_X, Constants.Shooter.Turret.OFFSET_POS_Y).rotateBy(new Rotation2d(getPose().getRotation().getRadians())), new Rotation2d(0)));
      double distance = 0.0;
      if (leftSide) {
        distance = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Math.sqrt(Math.pow((currentPose.getX() - BLUE_PASSING_LEFT_X), 2) + Math.pow((currentPose.getY() - BLUE_PASSING_LEFT_Y), 2)) : Math.sqrt(Math.pow((currentPose.getX() - RED_PASSING_LEFT_X), 2) + Math.pow((currentPose.getY() - RED_PASSING_LEFT_Y), 2));
      } else {
        distance = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Math.sqrt(Math.pow((currentPose.getX() - BLUE_PASSING_RIGHT_X), 2) + Math.pow((currentPose.getY() - BLUE_PASSING_RIGHT_Y), 2)) : Math.sqrt(Math.pow((currentPose.getX() - RED_PASSING_RIGHT_X), 2) + Math.pow((currentPose.getY() - RED_PASSING_RIGHT_Y), 2));
      }
      dist = distance;
      distanceLogged.set(distance);
      if (distance > 5.5) return flywheelSpeedsPASSING.get(distance);
      return flywheelSpeeds.get(distance);
    }

    public double calculateHoodAnglePassing(boolean leftSide) {
      Pose2d currentPose = getPose(); 
      currentPose = currentPose.plus(new Transform2d(new Translation2d(Constants.Shooter.Turret.OFFSET_POS_X, Constants.Shooter.Turret.OFFSET_POS_Y).rotateBy(new Rotation2d(getPose().getRotation().getRadians())), new Rotation2d(0)));
      double distance = 0.0;
      if (leftSide) {
        distance = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Math.sqrt(Math.pow((currentPose.getX() - BLUE_PASSING_LEFT_X), 2) + Math.pow((currentPose.getY() - BLUE_PASSING_LEFT_Y), 2)) : Math.sqrt(Math.pow((currentPose.getX() - RED_PASSING_LEFT_X), 2) + Math.pow((currentPose.getY() - RED_PASSING_LEFT_Y), 2));
      } else {
        distance = DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? Math.sqrt(Math.pow((currentPose.getX() - BLUE_PASSING_RIGHT_X), 2) + Math.pow((currentPose.getY() - BLUE_PASSING_RIGHT_Y), 2)) : Math.sqrt(Math.pow((currentPose.getX() - RED_PASSING_RIGHT_X), 2) + Math.pow((currentPose.getY() - RED_PASSING_RIGHT_Y), 2));
      }
      distanceLogged.set(distance);
      if (distance > 5.5) return hoodAnglesPASSING.get(distance);
      return hoodAngles.get(distance);
    }

    public int getPassingSide() {
    //Return 1 means left side
    //Return 0 means right side
    //Return -1 means middle
    double robotYPos = getPose().getY();
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      //Red
      if (robotYPos >= 0.0 && robotYPos <= 3.5) {
        return 1;
      } else if (robotYPos >= 4.7 && robotYPos <= 8.1) {
        return 0;
      }
    } else {
      //Blue 
      if (robotYPos >= 0.0 && robotYPos <= 3.5) {
        return 0;
      } else if (robotYPos >= 4.7 && robotYPos <= 8.1) {
        return 1;
      }
    }
    return -1;
  }

  public boolean fuelInVision() {
    return (taSubscriber.get() > 0.5 && txSubscriber.get() != 0 && tySubscriber.get() != 0);
  }

  public Translation2d getVelocity() {
    return new Translation2d(getChassisSpeeds().vxMetersPerSecond, getChassisSpeeds().vyMetersPerSecond);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public double getTOF(double distance) {
    return timeOfFlight.get(distance);
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
