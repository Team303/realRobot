package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

import static frc.robot.RobotContainer.controller;
import static frc.robot.RobotContainer.drive;
// import static frc.robot.RobotContainer.tempDrivebase;
import static frc.robot.RobotContainer.opController;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class Turret extends SubsystemBase {

  private final TalonFX turretMotor;
  private final CANcoder throughBore; // Powered by CANCoder

  private final LoggedNetworkNumber motorPosition;
  private final LoggedNetworkNumber throughBorePosition;

  public static LoggedTunableNumber TESTING_kP =
      new LoggedTunableNumber("TURRET TESTING_kP", Constants.Shooter.Turret.TURRET_kP);
  public static LoggedTunableNumber TESTING_kI =
      new LoggedTunableNumber("TURRET TESTING_kI", Constants.Shooter.Turret.TURRET_kI);
  public static LoggedTunableNumber TESTING_kD =
      new LoggedTunableNumber("TURRET TESTING_kD", Constants.Shooter.Turret.TURRET_kD);
  public static LoggedTunableNumber TESTING_kA =
      new LoggedTunableNumber("TURRET TESTING_kA", Constants.Shooter.Turret.TURRET_kA);
  public static LoggedTunableNumber TESTING_mmV =
      new LoggedTunableNumber("TURRET TESTING_mmV", Constants.Shooter.Turret.TURRET_maxV);
  public static LoggedTunableNumber TESTING_mmA =
      new LoggedTunableNumber("TURRET TESTING_mmA", Constants.Shooter.Turret.TURRET_maxA);

  public static LoggedTunableNumber GOAL_POS = new LoggedTunableNumber("TURRET GOAL_POS", Constants.Shooter.Turret.TEST_HUB_POS);

  private static int wallahi;

  private static final double BLUE_HUB_X = 4.6269;
  private static final double BLUE_HUB_Y = 4.03;
  private static final double RED_HUB_X = 11.91358;
  private static final double RED_HUB_Y = 4.03;

  private static final double BLUE_PASSING_LEFT_X = 3.063;
  private static final double BLUE_PASSING_LEFT_Y = 5.85;
  private static final double BLUE_PASSING_RIGHT_X = 3.063;
  private static final double BLUE_PASSING_RIGHT_Y = 2.15;

  private static final double RED_PASSING_LEFT_X = 13.500;
  private static final double RED_PASSING_LEFT_Y = 5.85;
  private static final double RED_PASSING_RIGHT_X = 14.670;
  private static final double RED_PASSING_RIGHT_Y = 2.15;

  public enum ALLIANCE_SHIFT {
    BLUE,
    RED,
    NONE
  }
  private static ALLIANCE_SHIFT previousShift = ALLIANCE_SHIFT.NONE;
  private static ALLIANCE_SHIFT currentShift = ALLIANCE_SHIFT.NONE;

  private static LoggedNetworkString CurrentAllianceShiftLogged = new LoggedNetworkString("Alliance Shift", "N/A");


  public Turret() {
    throughBore = new CANcoder(Constants.Shooter.Turret.TURRET_THROUGHBORE_ID, "topside");
    CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
    cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    cc_cfg.MagnetSensor.MagnetOffset = 0.0;
    throughBore.getConfigurator().apply(cc_cfg);

    turretMotor = new TalonFX(Constants.Shooter.Turret.TURRET_MOTOR_ID, "topside");

    turretMotor.setPosition(throughBore.getAbsolutePosition().getValueAsDouble());

    applyMainConfigs();

    throughBorePosition = new LoggedNetworkNumber("Turret Absolute Position", 0.0);
    motorPosition = new LoggedNetworkNumber("Turret Motor Position", 0.0);
    
    wallahi = 0;
  }

  public void printRAWPositionForOffset() {
    CANcoderConfiguration tempConfiguration = new CANcoderConfiguration();
    tempConfiguration.MagnetSensor.MagnetOffset = 0;
    tempConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0;
    throughBore.getConfigurator().apply(tempConfiguration);

    throughBore.getAbsolutePosition().waitForUpdate(0.1);
    double rawPosition = throughBore.getAbsolutePosition().getValueAsDouble();

    System.out.println(rawPosition);
  }

  private void applyMainConfigs() {
    var turretMotorConfig = new TalonFXConfiguration();

    turretMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretMotorConfig.Feedback.FeedbackRemoteSensorID = throughBore.getDeviceID();
    turretMotorConfig.Feedback.SensorToMechanismRatio = 8.5;
    //turretMotorConfig.Feedback.RotorToSensorRatio = 8.5;

    var Slot0Configs = turretMotorConfig.Slot0;
    Slot0Configs.kS = Constants.Shooter.Turret.TURRET_kS;
    Slot0Configs.kP = Constants.Shooter.Turret.TURRET_kP;
    Slot0Configs.kI = Constants.Shooter.Turret.TURRET_kI;
    Slot0Configs.kD = Constants.Shooter.Turret.TURRET_kD;
    Slot0Configs.kA = Constants.Shooter.Turret.TURRET_kA;

    var motionMagicConfigs = turretMotorConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Shooter.Turret.TURRET_maxV;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Shooter.Turret.TURRET_maxA;
    turretMotor.getConfigurator().apply(turretMotorConfig);

    var motorTalonFXConfigurator = turretMotor.getConfigurator();

    var limitConfigs = new CurrentLimitsConfigs();
    limitConfigs.StatorCurrentLimit = 30;
    limitConfigs.SupplyCurrentLimit = 30;
    limitConfigs.StatorCurrentLimitEnable = true;
    limitConfigs.SupplyCurrentLimitEnable = true;
    motorTalonFXConfigurator.apply(limitConfigs);

    var motorConfigs = new MotorOutputConfigs();
    motorConfigs.NeutralMode = NeutralModeValue.Brake;
    motorConfigs.Inverted = InvertedValue.Clockwise_Positive; // Change on testing
    motorTalonFXConfigurator.apply(motorConfigs);
  }

  public double getThroughPosition() {
    return throughBore.getAbsolutePosition().getValueAsDouble();
  }

  public double getMotorPosition() {
    return turretMotor.getPosition().getValueAsDouble();
  }

  //USE drive.getPose() to get
  public void moveToPos(double pos) {
    if (pos > 0.0 && pos <= Constants.Shooter.Turret.MAX_TURRET_ROTATION) { //Expand to 0.25 later
      final MotionMagicVoltage mmRequest = new MotionMagicVoltage(pos);
      turretMotor.setControl(mmRequest.withPosition(pos));
    } else {
      double newPos = 0.0;
      if (pos - (Constants.Shooter.Turret.MAX_TURRET_ROTATION / 2.0) > 0) {
        newPos = Constants.Shooter.Turret.MAX_TURRET_ROTATION;
      } else {
        newPos = 0.0;
      }
      final MotionMagicVoltage mmRequest = new MotionMagicVoltage(newPos);
      turretMotor.setControl(mmRequest.withPosition(newPos)); //Set to home to reset for a bit
    }
  }

  public void stopMotor() {
    turretMotor.setVoltage(0);
  }

  private double getBluePassingLeftRotate(Pose2d curPose) {
    double xDIFF = (BLUE_PASSING_LEFT_X) - curPose.getX();
    double yDIFF = (BLUE_PASSING_LEFT_Y) - curPose.getY();
    double radiansRotate = -Math.atan(yDIFF / xDIFF);

    double finalRadiansRotate = 0.0;

    if (curPose.getRotation().getRadians() < 0) {
      finalRadiansRotate = radiansRotate + (2 * Math.PI) + curPose.getRotation().getRadians();
    } else {
      finalRadiansRotate = radiansRotate + curPose.getRotation().getRadians();
    }
    double finalAngleRotate = Math.toDegrees(finalRadiansRotate);
    
    return (finalAngleRotate - 45);// * (0.46/0.5);
  }

  private double getBluePassingRightRotate(Pose2d curPose) {
    double xDIFF = (BLUE_PASSING_RIGHT_X) - curPose.getX();
    double yDIFF = (BLUE_PASSING_RIGHT_Y) - curPose.getY();
    double radiansRotate = -Math.atan(yDIFF / xDIFF);

    double finalRadiansRotate = 0.0;

    if (curPose.getRotation().getRadians() < 0) {
      finalRadiansRotate = radiansRotate + (2 * Math.PI) + curPose.getRotation().getRadians();
    } else {
      finalRadiansRotate = radiansRotate + curPose.getRotation().getRadians();
    }
    double finalAngleRotate = Math.toDegrees(finalRadiansRotate);

    return (finalAngleRotate - 45);
  }

  private double getRedPassingLeftRotate(Pose2d curPose) {
    double xDIFF = (RED_PASSING_LEFT_X) - curPose.getX();
    double yDIFF = (RED_PASSING_LEFT_Y) - curPose.getY();
    double radiansRotate = -Math.atan(yDIFF / xDIFF);

    double finalRadiansRotate = radiansRotate + curPose.getRotation().getRadians();
    double finalAngleRotate = Math.toDegrees(finalRadiansRotate);

    return (finalAngleRotate + 135);// * (0.46/0.5);
  }

  private double getRedPassingRightRotate(Pose2d curPose) {
    double xDIFF = (RED_PASSING_RIGHT_X) - curPose.getX();
    double yDIFF = (RED_PASSING_RIGHT_Y) - curPose.getY();
    double radiansRotate = -Math.atan(yDIFF / xDIFF);

    double finalRadiansRotate = radiansRotate + curPose.getRotation().getRadians();
    double finalAngleRotate = Math.toDegrees(finalRadiansRotate);
    return (finalAngleRotate + 135);// * (0.46/0.5);
  }

  public int getPassingSide() {
    //Return 1 means left side
    //Return 0 means right side
    //Return -1 means middle
    double robotYPos = drive.getPose().getY();
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      //Red
      if (robotYPos >= 0.0 && robotYPos <= 3.8) {
        return 1;
      } else if (robotYPos >= 4.2 && robotYPos <= 8.1) {
        return 0;
      }
    } else {
      //Blue 
      if (robotYPos >= 0.0 && robotYPos <= 3.8) {
        return 0;
      } else if (robotYPos >= 4.2 && robotYPos <= 8.1) {
        return 1;
      }
    }
    return -1;
  }

  private double normalizeRedRot(double input) {
    double output = -(input - Math.PI);
    if (output > Math.PI) {
        output -= 2 * Math.PI;
    }
    return output;
  }

  private double getBlueHubRotate(Pose2d curPose) {
    double xDIFF = BLUE_HUB_X - curPose.getX();
    double yDIFF = BLUE_HUB_Y - curPose.getY();
    double radiansRotate = -Math.atan(yDIFF / xDIFF);

    double finalRadiansRotate = radiansRotate + curPose.getRotation().getRadians();
    double finalAngleRotate = Math.toDegrees(finalRadiansRotate);
    return (finalAngleRotate + 135);
  }

  private double getRedHubRotate(Pose2d curPose) {
    double xDIFF = RED_HUB_X - curPose.getX();
    double yDIFF = RED_HUB_Y - curPose.getY(); 
    double radiansRotate = -Math.atan(yDIFF / xDIFF);
    
    double finalRadiansRotate = radiansRotate - normalizeRedRot(curPose.getRotation().getRadians());
    double finalAngleRotate = Math.toDegrees(finalRadiansRotate);

    return finalAngleRotate + 135; 
  }

  public double getTurretTurnPos() {
    Pose2d currentPose = drive.getPose(); 
    currentPose = currentPose.plus(new Transform2d(new Translation2d(Constants.Shooter.Turret.OFFSET_POS_X, Constants.Shooter.Turret.OFFSET_POS_Y)
                                                   .rotateBy(new Rotation2d(drive.getPose().getRotation().getRadians())), new Rotation2d(0)));
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) return getRedHubRotate(currentPose);
    return getBlueHubRotate(currentPose);
  }

  public double getTurretTurnPos(Pose2d pose) {
  //  Pose2d currentPose = drive.getPose(); //NEED TO ADD OFFSET FOR TURRET POSITION!!!
  //  currentPose = currentPose.plus(new Transform2d(new Translation2d(Constants.Shooter.Turret.OFFSET_POS_X, Constants.Shooter.Turret.OFFSET_POS_Y).rotateBy(new Rotation2d(drive.getPose().getRotation().getRadians())), new Rotation2d(0)));
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) return getRedHubRotate(pose);
    return getBlueHubRotate(pose);
  }

  public double getTurretPassingPos(boolean leftSide) {
    Pose2d currentPose = drive.getPose(); //NEED TO ADD OFFSET FOR TURRET POSITION!!!
    currentPose = currentPose.plus(new Transform2d(new Translation2d(Constants.Shooter.Turret.OFFSET_POS_X, Constants.Shooter.Turret.OFFSET_POS_Y).rotateBy(new Rotation2d(drive.getPose().getRotation().getRadians())), new Rotation2d(0)));
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) return leftSide ? getRedPassingLeftRotate(currentPose) : getRedPassingRightRotate(currentPose);
    return leftSide ? getBluePassingLeftRotate(currentPose) : getBluePassingRightRotate(currentPose);
  }

  @Override
  public void periodic() {
    //printRAWPositionForOffset();
    throughBorePosition.set(getThroughPosition());
    motorPosition.set(getMotorPosition());
    //System.out.println("GOAL: " + Constants.Shooter.Turret.TURRET_HOME_POS + "; END: " + getMotorPosition() + "; DIFF" + Math.abs(Constants.Shooter.Turret.TURRET_HOME_POS - getMotorPosition()));
    
    String gameData = "";
    gameData = DriverStation.getGameSpecificMessage();
    //System.out.println("DATA: " + gameData);
      if (gameData.length() > 0) {
      
      switch (gameData.charAt(0)) {
        case 'B':
          currentShift = ALLIANCE_SHIFT.BLUE;
          CurrentAllianceShiftLogged.set("BLUE ALLIANCE");
          break;
        case 'R':
          currentShift = ALLIANCE_SHIFT.RED;
          CurrentAllianceShiftLogged.set("RED ALLIANCE");
        default:
          CurrentAllianceShiftLogged.set(gameData);
          break;
      }

      if (currentShift != previousShift && previousShift != ALLIANCE_SHIFT.NONE) {
        //RUMBLE
        controller.setRumble(RumbleType.kBothRumble, 0.25);
        opController.setRumble(RumbleType.kBothRumble, 0.25);
        wallahi++;
        if (wallahi > 25) {
          previousShift = currentShift;
          wallahi = 0;
        }
      } else {
        //SAME SHIFT
      }
    }
    
  }

  
  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
