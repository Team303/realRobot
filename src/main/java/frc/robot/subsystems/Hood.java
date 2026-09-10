package frc.robot.subsystems;

import static frc.robot.RobotContainer.drive;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.LoggedTunableNumber;

public class Hood extends SubsystemBase{
    public final TalonFX hoodMotor;
    private final CANcoder hoodCaNcoder;

    private final LoggedNetworkNumber motorPosition;
  private final LoggedNetworkNumber throughBorePosition;
  public final LoggedNetworkNumber hoodFoundPos;

   private static final double BLUE_HUB_X = 4.6269;
  private static final double BLUE_HUB_Y = 4.03;
  private static final double RED_HUB_X = 11.91358;
  private static final double RED_HUB_Y = 4.03;



  public static LoggedTunableNumber TESTING_kP =
      new LoggedTunableNumber("TURRET TESTING_kP", Constants.Shooter.Hood.HOOD_kP);
  public static LoggedTunableNumber TESTING_kI =
      new LoggedTunableNumber("TURRET TESTING_kI", Constants.Shooter.Hood.HOOD_kI);
  public static LoggedTunableNumber TESTING_kD =
      new LoggedTunableNumber("TURRET TESTING_kD", Constants.Shooter.Hood.HOOD_kD);
  public static LoggedTunableNumber TESTING_kA =
      new LoggedTunableNumber("TURRET TESTING_kA", Constants.Shooter.Hood.HOOD_kA);
  public static LoggedTunableNumber TESTING_mmV =
      new LoggedTunableNumber("TURRET TESTING_mmV", Constants.Shooter.Hood.HOOD_maxV);
  public static LoggedTunableNumber TESTING_mmA =
      new LoggedTunableNumber("TURRET TESTING_mmA", Constants.Shooter.Hood.HOOD_maxA);

    public static LoggedTunableNumber HOOD_GOAL_POS = new LoggedTunableNumber("HOOD_GOAL_POS", 0.0);
    public static LoggedTunableNumber HOOD_INTERP_POS = new LoggedTunableNumber("HOOD_INTERP_POS", 0.0);
    
    public static InterpolatingDoubleTreeMap hoodAngles;


    public Hood() {
        hoodMotor = new TalonFX(Constants.Shooter.Hood.HOOD_MOTOR_ID, "topside");
        hoodCaNcoder = new CANcoder(Constants.Shooter.Hood.HOOD_THROUGHBORE_ID, "topside");

        CANcoderConfiguration cc_cfg = new CANcoderConfiguration();
        cc_cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0;
        cc_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;


        var hoodMotorConfig = new TalonFXConfiguration();

        // hoodMotorConfig.Feedback.FeedbackRemoteSensorID = hoodCaNcoder.getDeviceID();
        // hoodMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        // hoodMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
        // hoodMotorConfig.Feedback.RotorToSensorRatio = 1.0; //NEED TO CHANGE

        var Slot0Configs = hoodMotorConfig.Slot0;
        Slot0Configs.kG = Constants.Shooter.Hood.HOOD_kG;
        Slot0Configs.kS = Constants.Shooter.Hood.HOOD_kS;
        Slot0Configs.kP = Constants.Shooter.Hood.HOOD_kP;
        Slot0Configs.kI = Constants.Shooter.Hood.HOOD_kI;
        Slot0Configs.kD = Constants.Shooter.Hood.HOOD_kD;

        var motionMagicConfigs = hoodMotorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Shooter.Hood.HOOD_maxV;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Shooter.Hood.HOOD_maxA;
        hoodMotor.getConfigurator().apply(hoodMotorConfig);


      //  hoodMotor.setPosition(hoodCaNcoder.getAbsolutePosition().getValueAsDouble());
        hoodMotor.setPosition(0);
        var motorTalonFXConfigurator = hoodMotor.getConfigurator();

        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = 20;
        limitConfigs.SupplyCurrentLimit = 20;
        limitConfigs.StatorCurrentLimitEnable = true;
        limitConfigs.SupplyCurrentLimitEnable = true;
        motorTalonFXConfigurator.apply(limitConfigs);

        var motorConfigs = new MotorOutputConfigs();
        motorConfigs.NeutralMode = NeutralModeValue.Brake;
        motorConfigs.Inverted = InvertedValue.Clockwise_Positive; // Change on testing
        motorTalonFXConfigurator.apply(motorConfigs);


        //0.326
        throughBorePosition = new LoggedNetworkNumber("Hood Absolute Position", 0.0);
        motorPosition = new LoggedNetworkNumber("Hood Motor Position", 0.0);
        hoodFoundPos = new LoggedNetworkNumber("Hood Found Pos", 0.0);
        hoodAngles = new InterpolatingDoubleTreeMap();
        //Distance and Angle
        hoodAngles.put(1.6002, 0.0);
        hoodAngles.put(2.3876, 0.1);
        hoodAngles.put(2.4638, 0.2); 
        hoodAngles.put(3.2766, 0.35);
        hoodAngles.put(3.7338, 0.48);
        hoodAngles.put(4.1148, 0.6); 
        hoodAngles.put(4.4196, 0.65);  
        hoodAngles.put(4.9550, 1.0);
        
        
        //Theortical Passing ones
        hoodAngles.put(5.5, 1.05);
        hoodAngles.put(6.0, 1.15);
        hoodAngles.put(6.5, 1.2);
        hoodAngles.put(7.0, 1.2);
        hoodAngles.put(7.5, 1.2);
        hoodAngles.put(8.0, 1.2);
        hoodAngles.put(8.5, 1.2);
        hoodAngles.put(9.0, 1.2);

    }   

    // public double calculateHoodAngle(Pose2d robotPose) {
    //    double robotPoseX = drive.getPose().getX();
    //     double robotPoseY = robotPose.getY();
    //     double distance = Math.sqrt(Math.pow((robotPoseX - BLUE_HUB_X), 2) + Math.pow((robotPoseY - BLUE_HUB_Y), 2));
    //     System.out.println("robotpose: " + robotPose);
    //     return hoodAngles.get(distance);
    // }
    
    public void createNewConfig() {
    var hoodMotorConfig = new TalonFXConfiguration();

    //hoodMotorConfig.Feedback.FeedbackRemoteSensorID = throughBore.getDeviceID();
    //hoodMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    hoodMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
    hoodMotorConfig.Feedback.RotorToSensorRatio = 1.0; //NEED TO CHANGE

    var Slot0Configs = hoodMotorConfig.Slot0;
    //Slot0Configs.kS = Constants.Shooter.Hood.HOOD_kS;
    Slot0Configs.kP = Constants.Shooter.Hood.HOOD_kP;
    Slot0Configs.kI = Constants.Shooter.Hood.HOOD_kI;
    Slot0Configs.kD = Constants.Shooter.Hood.HOOD_kD;

    var motionMagicConfigs = hoodMotorConfig.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Shooter.Hood.HOOD_maxV;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Shooter.Hood.HOOD_maxA;
    hoodMotor.getConfigurator().apply(hoodMotorConfig);
  }

    public double getThroughPosition() {
    return hoodCaNcoder.getAbsolutePosition().getValueAsDouble();
  }

  public double getMotorPosition() {
    return hoodMotor.getPosition().getValueAsDouble();
  }

    public void moveToPos(double pos) {
        final MotionMagicVoltage mmRequest = new MotionMagicVoltage(pos);
        hoodMotor.setControl(mmRequest.withPosition(pos));
    }

    public void stopMotors() {
        hoodMotor.set(0);
    }

    @Override
  public void periodic() {
    throughBorePosition.set(getThroughPosition());
    motorPosition.set(getMotorPosition());
   // System.out.println("HOOD interp: " + hoodAngles.get(HOOD_INTERP_POS.getAsDouble()));
  //  System.out.println(hoodMotor.getPosition().getValueAsDouble());
  }

}
