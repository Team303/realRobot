package frc.robot.subsystems;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import frc.robot.util.LoggedTunableNumber;

public class Flywheel extends SubsystemBase {
    public final TalonFX leftFlywheelMotor;
    public final TalonFX rightFlywheelMotor;
    public final TalonFX kickerMotor;

    public final InterpolatingDoubleTreeMap flywheelSpeeds;

    private final LoggedNetworkNumber leftMotorSpeed;
    private final LoggedNetworkNumber rightMotorSpeed;

    public final LoggedNetworkNumber flywheelInterpNumber;

    private static final double BLUE_HUB_X = 4.6269;
  private static final double BLUE_HUB_Y = 4.03;
  private static final double RED_HUB_X = 11.91358;
  private static final double RED_HUB_Y = 4.03;

    public static LoggedTunableNumber FLY_TESTING_kP =
        new LoggedTunableNumber("FLYWHEEL TESTING_kP", Constants.Shooter.Flywheel.FLYWHEEL_kP);
    public static LoggedTunableNumber FLY_TESTING_kI =
        new LoggedTunableNumber("FLYWHEEL TESTING_kI", Constants.Shooter.Flywheel.FLYWHEEL_kI);
    public static LoggedTunableNumber FLY_TESTING_kD =
        new LoggedTunableNumber("FLYWHEEL TESTING_kD", Constants.Shooter.Flywheel.FLYWHEEL_kD);
    public static LoggedTunableNumber FLY_TESTING_kV =
        new LoggedTunableNumber("FLYWHEEL TESTING_kV", Constants.Shooter.Flywheel.FLYWHEEL_kV);
    public static LoggedTunableNumber FLY_TESTING_mmV =
        new LoggedTunableNumber("FLYWHEEL TESTING_mmV", Constants.Shooter.Flywheel.FLYWHEEL_maxV);
    public static LoggedTunableNumber FLY_TESTING_mmA =
        new LoggedTunableNumber("FLYWHEEL TESTING_mmA", Constants.Shooter.Flywheel.FLYWHEEL_maxA);

    public static LoggedTunableNumber GOAL_SPEED = new LoggedTunableNumber("FLYWHEEL GOAL_SPEED", 0);
    public static LoggedTunableNumber FLYWHEEL_INTERP_GOAL = new LoggedTunableNumber("FLYWHEEL_INTERP_GOAL", 0);

    public Flywheel() {
        leftFlywheelMotor = new TalonFX(Constants.Shooter.Flywheel.FLYWHEEL_LEFT_MOTOR_ID, "topside");
        rightFlywheelMotor = new TalonFX(Constants.Shooter.Flywheel.FLYWHEEL_RIGHT_MOTOR_ID, "topside");
        kickerMotor = new TalonFX(Constants.Spindexer.KICKER_MOTOR_ID, "topside");
        var flywheelMotorsConfig = new TalonFXConfiguration();

        var Slot0Configs = flywheelMotorsConfig.Slot0;
        Slot0Configs.kS = Constants.Shooter.Flywheel.FLYWHEEL_kS;
        Slot0Configs.kP = Constants.Shooter.Flywheel.FLYWHEEL_kP;
        Slot0Configs.kI = Constants.Shooter.Flywheel.FLYWHEEL_kI;
        Slot0Configs.kD = Constants.Shooter.Flywheel.FLYWHEEL_kD;
        Slot0Configs.kV = Constants.Shooter.Flywheel.FLYWHEEL_kV;


        var Slot1Configs = flywheelMotorsConfig.Slot1;
        Slot1Configs.kS = 1;
        Slot1Configs.kP = 1;
        Slot1Configs.kI = 0.0;
        Slot1Configs.kD = 0.0;
        Slot1Configs.kV = 0.13;


        var motionMagicConfigs = flywheelMotorsConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Shooter.Flywheel.FLYWHEEL_maxV;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Shooter.Flywheel.FLYWHEEL_maxA;

        leftFlywheelMotor.getConfigurator().apply(Slot0Configs); //MASTER MOTOR
        leftFlywheelMotor.getConfigurator().apply(motionMagicConfigs);

        rightFlywheelMotor.getConfigurator().apply(Slot0Configs); //MASTER MOTOR
        rightFlywheelMotor.getConfigurator().apply(motionMagicConfigs);

        kickerMotor.getConfigurator().apply(Slot1Configs); //MASTER MOTOR
        kickerMotor.getConfigurator().apply(motionMagicConfigs);

        var limitConfigs = new CurrentLimitsConfigs();
        limitConfigs.StatorCurrentLimit = 120;
        limitConfigs.SupplyCurrentLimit = 120;
        limitConfigs.StatorCurrentLimitEnable = true;
        limitConfigs.SupplyCurrentLimitEnable = true;
        leftFlywheelMotor.getConfigurator().apply(limitConfigs);
        rightFlywheelMotor.getConfigurator().apply(limitConfigs);

        var limitConfigs2 = new CurrentLimitsConfigs();
        limitConfigs2.StatorCurrentLimit = 120;
        limitConfigs2.SupplyCurrentLimit = 120;
        limitConfigs2.StatorCurrentLimitEnable = true;
        limitConfigs2.SupplyCurrentLimitEnable = true;
        kickerMotor.getConfigurator().apply(limitConfigs2);

        var leftMotorConfigs = new MotorOutputConfigs();
        leftMotorConfigs.NeutralMode = NeutralModeValue.Brake;
        leftMotorConfigs.Inverted = InvertedValue.CounterClockwise_Positive; //NEED TO CHANGE WHEN SETUP
        leftFlywheelMotor.getConfigurator().apply(leftMotorConfigs);

        var rightMotorConfigs = new MotorOutputConfigs();
        rightMotorConfigs.NeutralMode = NeutralModeValue.Brake;
        rightMotorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        rightFlywheelMotor.getConfigurator().apply(rightMotorConfigs);

        var kickerMotorConfigs = new MotorOutputConfigs();
        kickerMotorConfigs.NeutralMode = NeutralModeValue.Brake;
        kickerMotorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
        kickerMotor.getConfigurator().apply(kickerMotorConfigs);

        leftMotorSpeed = new LoggedNetworkNumber("LEFT FLYWHEEL SPEED", 0.0);
        rightMotorSpeed = new LoggedNetworkNumber("RIGHT FLYWHEEL SPEED", 0.0);
        flywheelInterpNumber = new LoggedNetworkNumber("Flywheel Speed Found", 0.0);
        FLY_TESTING_kP = new LoggedTunableNumber("FLYWHEEL TESTING_kP", Constants.Shooter.Flywheel.FLYWHEEL_kP);



        flywheelSpeeds = new InterpolatingDoubleTreeMap();
        //Distance and Speed
        flywheelSpeeds.put(1.6002, -33.0);
        flywheelSpeeds.put(2.3876, -36.0);
        flywheelSpeeds.put(2.4638, -34.0);  
        flywheelSpeeds.put(3.2766, -37.0);
        flywheelSpeeds.put(3.7338, -39.75);
        flywheelSpeeds.put(4.1148, -41.5); 
        flywheelSpeeds.put(4.4196, -42.5);  
        flywheelSpeeds.put(4.9550, -46.75);
        
        //Theortical Passing ones
        flywheelSpeeds.put(5.5, -50.5);
        flywheelSpeeds.put(6.0, -53.513);
        flywheelSpeeds.put(6.5, -58.395);
        flywheelSpeeds.put(7.0, -61.23);
        flywheelSpeeds.put(7.5, -63.81);
        flywheelSpeeds.put(8.0, -65.52010);
        flywheelSpeeds.put(8.5, -66.45);
        flywheelSpeeds.put(9.0, -67.267);



    }


    public void createNewConfig() {
        var flywheelMotorsConfig = new TalonFXConfiguration();

        var Slot0Configs = flywheelMotorsConfig.Slot0;
        Slot0Configs.kS = Constants.Shooter.Flywheel.FLYWHEEL_kS;
        Slot0Configs.kP = FLY_TESTING_kP.getAsDouble();
        Slot0Configs.kI = FLY_TESTING_kI.getAsDouble();
        Slot0Configs.kD = FLY_TESTING_kD.getAsDouble();
        Slot0Configs.kV = FLY_TESTING_kV.getAsDouble();

        var motionMagicConfigs = flywheelMotorsConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = FLY_TESTING_mmV.getAsDouble();;
        motionMagicConfigs.MotionMagicAcceleration = FLY_TESTING_mmA.getAsDouble();;

        leftFlywheelMotor.getConfigurator().apply(flywheelMotorsConfig); //MASTER MOTOR
        rightFlywheelMotor.getConfigurator().apply(flywheelMotorsConfig);
    }

    public double getLeftMotorSpeed() {
        return leftFlywheelMotor.getVelocity().getValueAsDouble();
    }

    public double getRightMotorSpeed() {
        return rightFlywheelMotor.getVelocity().getValueAsDouble();
    }

    public double getRightMotorAccel() {
        return rightFlywheelMotor.getAcceleration().getValueAsDouble();
    }

    public double getLeftMotorAccel() {
        return leftFlywheelMotor.getAcceleration().getValueAsDouble();
    }

    public double getKickerMotorSpeed() {
        return kickerMotor.getVelocity().getValueAsDouble();
    }

    public double getKickerMotorAccel() {
        return kickerMotor.getAcceleration().getValueAsDouble();
    }

    public void getToSpeed(double speed) {
        final MotionMagicVelocityVoltage mmRequest = new MotionMagicVelocityVoltage(speed);
        final MotionMagicVelocityTorqueCurrentFOC mmRequest2 = new MotionMagicVelocityTorqueCurrentFOC(speed);
        mmRequest.EnableFOC = false;
        //leftFlywheelMotor.setControl(mmRequest);
        rightFlywheelMotor.setControl(mmRequest.withVelocity(speed));//new Follower(Constants.Shooter.Flywheel.FLYWHEEL_LEFT_MOTOR_ID, 
                                                 // MotorAlignmentValue.Opposed));
        leftFlywheelMotor.setControl(new Follower(Constants.Shooter.Flywheel.FLYWHEEL_RIGHT_MOTOR_ID,
                                                  MotorAlignmentValue.Opposed));


      //  kickerMotor.setControl(mmRequest.withVelocity(-speed*1.0).withSlot(1));
    kickerMotor.setControl(mmRequest.withVelocity(-speed*1.0).withSlot(1));

    }

    public void stopMotors() {
        leftFlywheelMotor.setVoltage(0);
        rightFlywheelMotor.setVoltage(0);
        kickerMotor.setVoltage(0);
    }

    

    @Override
    public void periodic() {
       leftMotorSpeed.set(getLeftMotorSpeed());
        rightMotorSpeed.set(getRightMotorSpeed());
        //System.out.println("FLYWHEEL: " + flywheelSpeeds.get(FLYWHEEL_INTERP_GOAL.getAsDouble()));
        //System.out.println("kicker speed: " + getKickerMotorSpeed());
        // System.out.println(getLeftMotorSpeed());
        // System.out.println(getRightMotorSpeed());
        //Logger.recordOutput("Flywheel/LeftMotorSpeed", getLeftMotorSpeed());
        //Logger.recordOutput("Flywheel/RightMotorSpeed", getRightMotorSpeed());
        //Logger.recordOutput("Flywheel/LeftMotorVoltage", leftFlywheelMotor.getMotorVoltage().getValueAsDouble());
        //Logger.recordOutput("Flywheel/RightMotorVoltage", rightFlywheelMotor.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation

    }
}