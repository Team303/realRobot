package frc.robot.subsystems;


import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Spindexer extends SubsystemBase{
    public TalonFX spindexerMotor;

    public Spindexer(){
        spindexerMotor = new TalonFX(Constants.Spindexer.SPINDEXER_MOTOR_ID, "topside");


        // leftElevatorMotor.setInverted(true);
    // rightElevatorMotor.setInverted(false);
    var TalonFXConfiguration = new TalonFXConfiguration();

    

    // var motionMagicConfigs = TalonFXConfiguration.MotionMagic;
    // motionMagicConfigs.MotionMagicAcceleration = 35;
    // motionMagicConfigs.MotionMagicCruiseVelocity = 60;
    spindexerMotor.getConfigurator().apply(TalonFXConfiguration);

    spindexerMotor.getConfigurator().apply(TalonFXConfiguration);

    var SpindexerMotorTalonFXConfigurator = spindexerMotor.getConfigurator();

    var limitConfigs = new CurrentLimitsConfigs();
    var spindexerMotorConfigs = new MotorOutputConfigs();

    spindexerMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
    spindexerMotorConfigs.NeutralMode = NeutralModeValue.Brake;

    // enable stator current limit
    limitConfigs.StatorCurrentLimit = 120;
    limitConfigs.StatorCurrentLimitEnable = true;

    limitConfigs.SupplyCurrentLimit = 120;
    limitConfigs.SupplyCurrentLimitEnable = true;

    SpindexerMotorTalonFXConfigurator.apply(spindexerMotorConfigs);
    SpindexerMotorTalonFXConfigurator.apply(limitConfigs);


   
    }
}
