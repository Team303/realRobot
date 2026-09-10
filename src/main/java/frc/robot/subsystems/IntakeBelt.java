package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class IntakeBelt extends SubsystemBase{
    public TalonFX beltMotor;
    public TalonFX takeMotor;
    public IntakeBelt () {
        beltMotor = new TalonFX(Constants.IntakeBelt.INTAKEBELT_MOTOR_ID, "topside");
        takeMotor = new TalonFX(Constants.IntakeBelt.TAKE_MOTOR_ID, "topside");
        var TalonFXConfiguration = new TalonFXConfiguration();

        // var motionMagicConfigs = TalonFXConfiguration.MotionMagic;
        // motionMagicConfigs.MotionMagicAcceleration = 35;
        // motionMagicConfigs.MotionMagicCruiseVelocity = 60;
        beltMotor.getConfigurator().apply(TalonFXConfiguration);

        var beltMotorTalonFXConfigurator = beltMotor.getConfigurator();

        var limitConfigs = new CurrentLimitsConfigs();
        var beltMotorConfigs = new MotorOutputConfigs();

        beltMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        beltMotorConfigs.NeutralMode = NeutralModeValue.Brake;

        // enable stator current limit
        limitConfigs.StatorCurrentLimit = 70; //belt stuff
        limitConfigs.StatorCurrentLimitEnable = true;

        limitConfigs.SupplyCurrentLimit = 70;
        limitConfigs.SupplyCurrentLimitEnable = true;

        beltMotorTalonFXConfigurator.apply(beltMotorConfigs);
        beltMotorTalonFXConfigurator.apply(limitConfigs);




        takeMotor.getConfigurator().apply(TalonFXConfiguration);



        var takeMotorTalonFXConfigurator = takeMotor.getConfigurator();

        var limitConfigs2 = new CurrentLimitsConfigs();
        var takeMotorConfigs = new MotorOutputConfigs();

        takeMotorConfigs.Inverted = InvertedValue.Clockwise_Positive;
        takeMotorConfigs.NeutralMode = NeutralModeValue.Brake;

        // enable stator current limit
        limitConfigs2.StatorCurrentLimit = 30;
        limitConfigs2.StatorCurrentLimitEnable = true;

        limitConfigs2.SupplyCurrentLimit = 30;
        limitConfigs2.SupplyCurrentLimitEnable = true;

        takeMotorTalonFXConfigurator.apply(takeMotorConfigs);
        takeMotorTalonFXConfigurator.apply(limitConfigs2);

    }
}
