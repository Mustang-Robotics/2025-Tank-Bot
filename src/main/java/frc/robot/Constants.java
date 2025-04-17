// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

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
  }

  public class AutoConfig {
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
    public static final String kBottomCamera = "Main";
    //public static final String kTopCamera = "Top";
    //public static final String objectCamera = "USB_Camera";
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kBottomRobotToCam =
            new Transform3d(new Translation3d(0.1778, -0.3112, 0.3112), new Rotation3d(0, 0, .175));
    //public static final Transform3d kTopRobotToCam =
    //        new Transform3d(new Translation3d(0.0445, -0.2223, 1.0478), new Rotation3d(0, -.524, 0));
    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(.5, .5, 2);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(.25, .25, .5);
}
}
