// package frc.robot.subsystems;

// import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
// import com.ctre.phoenix6.configs.MotorOutputConfigs;
// import com.ctre.phoenix6.configs.Slot0Configs;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.controls.MotionMagicVoltage;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import com.ctre.phoenix6.signals.NeutralModeValue;
// import edu.wpi.first.networktables.GenericEntry;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

// public class Climber extends SubsystemBase {
//     public final TalonFX climbMotor;

//     public static final ShuffleboardTab CLIMBER_TAB = Shuffleboard.getTab("Climber");

//     public static final GenericEntry climbMotorRotations = CLIMBER_TAB
//             .add("climb motor rotations", 0)
//             .withPosition(5, 0)
//             .withSize(2, 1)
//             .getEntry();

//     public double position;

//     public Climber() {
//         climbMotor = new TalonFX(frc.robot.Constants.Climber.CLIMBER_MOTOR_ID);

//         // leftElevatorMotor.setInverted(true);
//         // rightElevatorMotor.setInverted(false);
//         var TalonFXConfiguration = new TalonFXConfiguration();

//         var Slot0Configs = TalonFXConfiguration.Slot0;
//         var Slot1Configs = TalonFXConfiguration.Slot1;
//         Slot0Configs.kS = 0;
//         Slot1Configs.kS = Slot0Configs.kS;
//         Slot0Configs.kG = 0;
//         Slot1Configs.kG = Slot0Configs.kG;
//         Slot0Configs.kV = 0;
//         Slot1Configs.kV = -Slot0Configs.kV;
//         Slot0Configs.kA = 0;
//         Slot1Configs.kA = Slot0Configs.kA;
//         Slot0Configs.kP = 0;
//         Slot1Configs.kP = Slot0Configs.kP;
//         Slot0Configs.kI = 0;
//         Slot1Configs.kI = Slot0Configs.kI;
//         Slot0Configs.kD = 0;
//         Slot1Configs.kD = Slot0Configs.kD;

//         var motionMagicConfigs = TalonFXConfiguration.MotionMagic;
//         motionMagicConfigs.MotionMagicAcceleration = 30;
//         motionMagicConfigs.MotionMagicCruiseVelocity = 30;
//         climbMotor.getConfigurator().apply(TalonFXConfiguration);

//         climbMotor.setPosition(0);
//         climbMotor.getConfigurator().apply(TalonFXConfiguration);

//         var climbMotorTalonFXConfigurator = climbMotor.getConfigurator();

//         var limitConfigs = new CurrentLimitsConfigs();
//         var climbMotorConfigs = new MotorOutputConfigs();
//         climbMotorConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
//         climbMotorConfigs.NeutralMode = NeutralModeValue.Brake;

//         // enable stator current limit
//         limitConfigs.StatorCurrentLimit = 120;
//         limitConfigs.StatorCurrentLimitEnable = true;

//         limitConfigs.SupplyCurrentLimit = 120;
//         limitConfigs.SupplyCurrentLimitEnable = true;

//         climbMotorTalonFXConfigurator.apply(climbMotorConfigs);
//         climbMotorTalonFXConfigurator.apply(climbMotorConfigs);

//         // Slot0Configs.kG = 0.09;
//         // Slot0Configs.kV = 11.2;
//         // Slot0Configs.kA = 0.01;
//         // Slot0Configs.kP = 1;
//         // Slot0Configs.kI = 0;
//         // Slot0Configs.kD = 0;

//         // var motionMagicConfigs = TalonFXConfiguration.MotionMagic;
//         // motionMagicConfigs.MotionMagicAcceleration = 2000;
//         // motionMagicConfigs.MotionMagicJerk = 0;
//         // motionMagicConfigs.MotionMagicCruiseVelocity = 200;

//         // leftElevatorMotor.setNeutralMode(NeutralModeValue.Brake);
//         // rightElevatorMotor.setNeutralMode(NeutralModeValue.Brake);

//         position = 0;
//     }

//     public void moveToSetpoint(double targetPos) {
//         position = targetPos;
//         final MotionMagicVoltage m_request = new MotionMagicVoltage(targetPos);
//         climbMotor.setControl(m_request.withPosition((targetPos)).withSlot(1));
//     }

//     public void update() {
//         // position = leftElevatorMotor.getClosedLoopReference().getValueAsDouble();
//     }

//     public double getRealPosition(TalonFX motor) {
//         return motor.getPosition().getValueAsDouble(); 
//     }

//     public void resetPosition() {
//         climbMotor.setPosition(0);
//     }

//     public boolean atSetpoint() {
//         return Math.abs((double) getRealPosition(climbMotor) - (double) position) < 0.25;
//     }

//     @Override
//     public void periodic() {
//         climbMotorRotations.setDouble(getRealPosition(climbMotor));
//     }
// }
