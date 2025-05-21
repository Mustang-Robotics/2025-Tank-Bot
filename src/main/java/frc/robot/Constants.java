// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

//This section is all of the classes that are being use in this section of the code.
package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public final class Constants {

  //constants related to the driver(s) and controllers
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0; //USB port of controller. Used in RobotContainer
    public static final double kDeadband = 0.1; //Amount of joystick movement it takes to start registering. Used in ArcadeDrive command
  }

  //constants related to the drivesystem
  public static final class DriveConstants {

    public static final int kLeftLeader = 31; //CAN ID of LeftLeader Motor. Used in DriveSubsystem
    public static final int kLeftFollower = 32; //CAN ID of LeftFollower Motor. Used in DriveSubsystem
    public static final int kRightLeader = 33; //CAN ID of RightLeader Motor. Used in DriveSubsystem
    public static final int kRightFollower = 34; //CAN ID of RightFollower Motor. Used in DriveSubsystem
    public static final int kStallCurrent = 30; //Maximum amount of current that will get sent to a motor. Used in Configs
    //The following numbers get used in calculations to help tune the speed of the drivetrain
    public static final double kP = 0.0465; //Proportional Gain See Constants for explanation
    public static final double kI = 0; //Integral Gain See Constants for explanation
    public static final double kD = 0; //Derivative Gain See Constants for explanation
    public static final double kTrackwidth = 0.557; //width from center to center of wheels measured in meters.
    public static final double kWheelRadiusMeters = Units.inchesToMeters(3); //other way to use meters but not have to convert from inches
    public static final double kWheelCircumferenceMeters = kWheelRadiusMeters * 2 * Math.PI;
    public static final double kNeoFreeSpeed = 5676; //RPMS of NEO motor. Can be looked up on REVRobotics
    public static final double kNeoKv = kNeoFreeSpeed / 12; //12 is for 12V of battery
    public static final double kDrivingMotorFreeSpeedRps = kNeoFreeSpeed / 60;
    public static final double kDrivingMotorReduction = 8.46; //Gear Ratio of gear box
    public static final double kDriveWheelFreeSpeed = 
      (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction; //calculates max speed of the wheel
    public static final double kMaxSpeedMetersPerSec = 3.2; //limiting the max speed of the robot, this value is specific to MinneTrials
    public static final double kVoltPerSpeed = kDrivingMotorReduction * 60 / (kNeoKv * 2 * Math.PI * kWheelRadiusMeters); //calculates how many volts it takes to move a specific speed

    
  }

  public static final class AutoConfig {
    public static final Vector<N3> qelems = VecBuilder.fill(0.0625, 0.0625, .0625);
    public static final Vector<N2> relems = VecBuilder.fill(.5, 1); 
    public static RobotConfig config;
      static {
      try{
        config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        //Handle exception as needed
        e.printStackTrace();
      }
    }
  }

  public static class Vision {
    public static final String kTagCamera = "Main";

    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToTagCam =
            new Transform3d(new Translation3d(0.3397, 0, 0.1873), new Rotation3d(0, 0.4887, 0));
    

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(.25, .25, .25);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(.125, .125, .125);
}
}
