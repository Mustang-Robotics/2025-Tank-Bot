// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.1;
  }

  public static final class DriveConstants {

    public static final int kLeftLeader = 31;
    public static final int kLeftFollower = 32;
    public static final int kRightLeader = 33;
    public static final int kRightFollower = 34;
    public static final int kStallCurrent = 30;
    public static final double kP = 0.0465;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kTrackwidth = 0.557;
    public static final double kWheelRadiusMeters = Units.inchesToMeters(3);
    public static final double kWheelCircumferenceMeters = kWheelRadiusMeters * 2 * Math.PI;
    public static final double kNeoFreeSpeed = 5676;
    public static final double kNeoKv = 5676 / 12;
    public static final double kDrivingMotorFreeSpeedRps = kNeoFreeSpeed / 60;
    public static final double kDrivingMotorReduction = 8.46;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
      / kDrivingMotorReduction;
    public static final double kMaxSpeedMetersPerSec = 3.2;
    public static final double kVoltPerSpeed = kDrivingMotorReduction * 60 / (kNeoKv * 2 * Math.PI * kWheelRadiusMeters);

    
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

    public static final AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(.5, .5, 2);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(.25, .25, .5);
}
}
