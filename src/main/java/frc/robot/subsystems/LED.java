// package frc.robot.subsystems;

 
// import static edu.wpi.first.units.Units.Degrees;

// import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

// import com.ctre.phoenix6.CANBus;
// import com.ctre.phoenix6.configs.CANdleConfiguration;
// import com.ctre.phoenix6.controls.EmptyAnimation;
// import com.ctre.phoenix6.controls.SolidColor;
// import com.ctre.phoenix6.hardware.CANdle;
// import com.ctre.phoenix6.signals.RGBWColor;
// import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
// import com.ctre.phoenix6.signals.StripTypeValue;

// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.util.Color;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import frc.robot.Constants.LED.AnimationType;

// public class LED extends SubsystemBase {
//     public static final int kSlot0StartIdx = 0;
//     public static final int kSlot0EndIdx = 8;

//     public static final int kSlot1StartIdx = 9;
//     public static final int kSlot1EndIdx = 83;

//     public final CANdle m_candle;// = new CANdle(Constants.LED.LED_ID, "topside");


//     public static final RGBWColor kGreen = new RGBWColor(0, 217, 0, 0);
//     public static final RGBWColor kWhite = new RGBWColor(Color.kWhite).scaleBrightness(0.5);
//     public static final RGBWColor kViolet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
//     public static final RGBWColor kRed = RGBWColor.fromHex("#D9000000").orElseThrow();
//     public static final RGBWColor kTeal = RGBWColor.fromHex("#008F80").orElseThrow();
//     public static final RGBWColor kTealStrip = RGBWColor.fromHex("#D050FF").orElseThrow();
  
//     public LED() {
//         m_candle = new CANdle(Constants.LED.LED_ID, "topside");
//         /* Configure CANdle */
//         var cfg = new CANdleConfiguration();
//         /* set the LED strip type and brightness */
//         cfg.LED.StripType = StripTypeValue.GRB;
//         cfg.LED.BrightnessScalar = 0.5;
//         /* disable status LED when being controlled */
//         cfg.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

//         m_candle.getConfigurator().apply(cfg);

//         // m_candle.setControl(new SolidColor(0, 3).withColor(kGreen));
//         // m_candle.setControl(new SolidColor(4, 7).withColor(kWhite));


//         /* clear all previous animations */
//         for (int i = 0; i < 8; ++i) {
//             m_candle.setControl(new EmptyAnimation(i));
//         }
//     }

//     // public static void setAnim0State(AnimationType type) {
//     //     m_anim0State = type;
//     // }

//     // public static void setAnim1State(AnimationType type) {
//     //     m_anim1State = type;
//     // }




// }