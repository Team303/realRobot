// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.LoggedTunableNumber;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final boolean tuningMode = true;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final String CAN_BUS_TOP_NAME = "topside";

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class DriveToPoseConstraints {
    public static double maxVelocityMPS = 1000;
    public static double maxAccelerationMPSSq = 30;
    public static double maxAngularVelocityRadPerSec = 30;
    public static double maxAngularAccelerationRadPerSecSq = 12;

    public static LoggedTunableNumber maxVelocityMPSScaler =
        new LoggedTunableNumber("DriveToPoseConstaints/maxVelocityMPSScaler", 1);
    public static LoggedTunableNumber maxAccelerationMPSSqScaler =
        new LoggedTunableNumber("DriveToPoseConstaints/maxAccelerationMPSSqScaler", 1);
    public static LoggedTunableNumber maxAngularVelocityRadPerSecScaler =
        new LoggedTunableNumber("DriveToPoseConstaints/maxAngularVelocityRadPerSecScaler", 1);
    public static LoggedTunableNumber maxAngularAccelerationRadPerSecSqScaler =
        new LoggedTunableNumber("DriveToPoseConstaints/maxAngularAccelerationRadPerSecSqScaler", 1);

    public static PathConstraints fastpathConstraints =
        new PathConstraints(
            maxVelocityMPS,
            maxAccelerationMPSSq,
            maxAngularVelocityRadPerSec,
            maxAngularAccelerationRadPerSecSq);

    public static PathConstraints slowpathConstraints =
        new PathConstraints(
            maxVelocityMPS * maxVelocityMPSScaler.get(),
            maxAccelerationMPSSq * maxAccelerationMPSSqScaler.get(),
            maxAngularVelocityRadPerSec * maxAngularVelocityRadPerSecScaler.get(),
            maxAngularAccelerationRadPerSecSq * maxAngularAccelerationRadPerSecSqScaler.get());

    public static void updateTunableNumbers() {
      slowpathConstraints =
          new PathConstraints(
              maxVelocityMPS * maxVelocityMPSScaler.get(),
              maxAccelerationMPSSq * maxAccelerationMPSSqScaler.get(),
              maxAngularVelocityRadPerSec * maxAngularVelocityRadPerSecScaler.get(),
              maxAngularAccelerationRadPerSecSq * maxAngularAccelerationRadPerSecSqScaler.get());
    }
  }

  public static class DriveToPoseStraight {

    public static double maxVelocityMPS = 1000;
    public static double maxAngularVelocityRadPerSec = 300;

    public static class XController {
      public static double kP = 5;
      public static double kD = 0;
      public static double tolerance = 0.01; // ~.4 inches
    }

    public static class YController {
      public static double kP = 5;
      public static double kD = 0;
      public static double tolerance = 0.01;
    }

    public static class ThetaController {
      public static double kP = 20;
      public static double kD = 0;
      public static double tolerance = Units.degreesToRadians(1);
    }
  }

  public static class Spindexer{
    public static final int SPINDEXER_MOTOR_ID = 24; //change
    public static final int KICKER_MOTOR_ID = 29; //change
  }

  public static class Climber {
    public static final int CLIMBER_MOTOR_ID = 20;
  }

  public static class IntakeBelt {
    public static final int INTAKEBELT_MOTOR_ID = 11;
    public static final int TAKE_MOTOR_ID = 28; 

  }

 

  public static class Shooter {
    // TURRET CONSTANTS
    public static class Turret {
      public static final int TURRET_THROUGHBORE_ID = 41; // NEED TO CHANGE
      public static final int TURRET_MOTOR_ID = 31; // NEED TO CHANGE
      public static final double TURRET_kS = 0.13;
      public static final double TURRET_kP = 80;
      public static final double TURRET_kI = 0.01;
      public static final double TURRET_kD = 0.03;
      public static final double TURRET_kA = 0.0;

      public static final double TURRET_maxV = 125;
      public static final double TURRET_maxA = 25;

      public static final double OFFSET_POS_Y = 0.1651;
      public static final double OFFSET_POS_X = 0.08255;

      public static final double TURRET_MOTOR_THROUGHBORE_RATIO = 2.8;

      public static final double MAX_TURRET_ROTATION = 0.53;
      public static final double HARD_MAX_TURRET_ROTATION = 0.5;

      public static final double TURRET_HOME_POS = 0;
      public static final double TEST_HUB_POS = 0;

      public static final double MAGNET_CANCODER_OFFSET = 0.483154296875;
    }
    // FLYWHEEL CONSTANTS --> Need to tune all PID 
    public static class Flywheel {
      public static final int FLYWHEEL_LEFT_MOTOR_ID = 38;
      public static final int FLYWHEEL_RIGHT_MOTOR_ID = 37;
      public static final double FLYWHEEL_kS = 0.08;
      public static final double FLYWHEEL_kP = 0.25;
      public static final double FLYWHEEL_kI = 0.0;
      public static final double FLYWHEEL_kD = 0.0;
      public static final double FLYWHEEL_kV = 0.13;


      public static final double FLYWHEEL_maxV = 50;
      public static final double FLYWHEEL_maxA = 9999;
    }
    // HOOD CONSTANTS --> NEED TO TUNE ALL PID
    public static class Hood {
      public static final int HOOD_THROUGHBORE_ID = 30; 
      public static final int HOOD_MOTOR_ID = 25;
      public static final double HOOD_kG = 0.6;
      public static final double HOOD_kP = 40;
      public static final double HOOD_kI = 0.0;
      public static final double HOOD_kD = 0.0;
      public static final double HOOD_kA = 0.0;
      public static final double HOOD_kS = 0.0;

      public static final double HOOD_maxV = 25;
      public static final double HOOD_maxA = 100;

      public static final double HOOD_HOME_POS = 0.0; // NEED TO CHANGE
      public static final double HOOD_MAX_POS = 1; // NEED TO CHANGE
      public static final double GOAL_POS = 0.0;
    }
    
  }

  public static class LED{
      public static final int LED_ID = 45; //change
      public static enum AnimationType{
        None,
        ColorFlow,
        Fire,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Strobe,
        Twinkle,
        TwinkleOff,
        Solid
      }
    }
}
