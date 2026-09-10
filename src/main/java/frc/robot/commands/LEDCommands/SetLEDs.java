// package frc.robot.commands.LEDCommands;

// import com.ctre.phoenix6.controls.ColorFlowAnimation;
// import com.ctre.phoenix6.controls.FireAnimation;
// import com.ctre.phoenix6.controls.RainbowAnimation;
// import com.ctre.phoenix6.controls.SolidColor;
// import com.ctre.phoenix6.controls.StrobeAnimation;
// import com.ctre.phoenix6.controls.TwinkleAnimation;
// import com.ctre.phoenix6.controls.TwinkleOffAnimation;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.LED.AnimationType;
// import frc.robot.subsystems.LED;

// import static frc.robot.subsystems.LED.kSlot0StartIdx;
// import static frc.robot.subsystems.LED.kSlot0EndIdx;
// import static frc.robot.subsystems.LED.kSlot1StartIdx;
// import static frc.robot.subsystems.LED.kSlot1EndIdx;
// import static frc.robot.subsystems.LED.kTeal;
// import static frc.robot.subsystems.LED.kTealStrip;


// public class SetLEDs extends Command{
//     AnimationType animation;
//     AnimationType animation2;

//     LED ledSubsystem;

//     public SetLEDs(LED led, AnimationType x, AnimationType y){
//         addRequirements(led);
//         animation = x;
//         animation2 = y;
//         ledSubsystem = led;
//     }

//     public void execute(){
//             switch (animation) {
//                 default:
//                 case ColorFlow:
//                     ledSubsystem.m_candle.setControl(
//                         new ColorFlowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
//                             .withColor(kTeal)
//                     );
//                     break;
//                 case Rainbow:
//                     ledSubsystem.m_candle.setControl(
//                         new RainbowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
//                     );
//                     break;
//                 case Twinkle:
//                     ledSubsystem.m_candle.setControl(
//                         new TwinkleAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
//                             .withColor(kTeal)
//                     );
//                     break;
//                 case TwinkleOff:
//                     ledSubsystem.m_candle.setControl(
//                         new TwinkleOffAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
//                             .withColor(kTeal)
//                     );
//                     break;
//                 case Fire:
//                     ledSubsystem.m_candle.setControl(
//                         new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
//                     );
//                     break;
//                 case Strobe:
//                     ledSubsystem.m_candle.setControl(
//                         new StrobeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0).withColor(kTeal).withFrameRate(2.5)
//                     );
//                     break;
//                 case Solid:
//                     ledSubsystem.m_candle.setControl(
//                         new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(kTeal)
//                     );
//                     break;
//             }







//             switch (animation2) {
//                 default:
//                 case ColorFlow:
//                     ledSubsystem.m_candle.setControl(
//                         new ColorFlowAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
//                             .withColor(kTealStrip)
//                     );
//                     break;
//                 case Rainbow:
//                     ledSubsystem.m_candle.setControl(
//                         new RainbowAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
//                     );
//                     break;
//                 case Twinkle:
//                     ledSubsystem.m_candle.setControl(
//                         new TwinkleAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
//                             .withColor(kTealStrip)
//                     );
//                     break;
//                 case TwinkleOff:
//                     ledSubsystem.m_candle.setControl(
//                         new TwinkleOffAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
//                             .withColor(kTealStrip)
//                     );
//                     break;
//                 case Fire:
//                     ledSubsystem.m_candle.setControl(
//                         new FireAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
//                     );
//                     break;
//                 case Strobe:
//                     ledSubsystem.m_candle.setControl(
//                         new StrobeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1).withColor(kTealStrip).withFrameRate(2.5)
//                     );
//                     break;
//                 case Solid:
//                     ledSubsystem.m_candle.setControl(
//                         new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(kTealStrip)
//                     );
//                     break;
//             }

//         }
//     }
