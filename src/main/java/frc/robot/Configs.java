package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.DriveConstants;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public final class Configs {
    public static final class DriveSystem {
        public static final SparkMaxConfig leftLeadConfig = new SparkMaxConfig();
        public static final SparkMaxConfig leftFollowConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rightLeadConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rightFollowConfig = new SparkMaxConfig();

        static {

            leftLeadConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(DriveConstants.kStallCurrent)
                .voltageCompensation(12);
            
            leftLeadConfig 
                .encoder
                .positionConversionFactor(2*Math.PI / DriveConstants.kDrivingMotorReduction)
                .velocityConversionFactor(2*Math.PI / 60 / DriveConstants.kDrivingMotorReduction)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

            leftLeadConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD)
                .velocityFF(1/DriveConstants.kDriveWheelFreeSpeedRps)
                .outputRange(-1, 1);

            leftFollowConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(DriveConstants.kStallCurrent)
                .voltageCompensation(12)
                .follow(DriveConstants.kLeftLeader);

            rightLeadConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(DriveConstants.kStallCurrent)
                .voltageCompensation(12)
                .inverted(true);
            
            rightLeadConfig 
                .encoder
                .positionConversionFactor(2*Math.PI / DriveConstants.kDrivingMotorReduction)
                .velocityConversionFactor(2*Math.PI / 60 / DriveConstants.kDrivingMotorReduction)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

            rightLeadConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD)
                .velocityFF(1/DriveConstants.kDriveWheelFreeSpeedRps)
                .outputRange(-1, 1);

            rightFollowConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(DriveConstants.kStallCurrent)
                .voltageCompensation(12)
                .follow(DriveConstants.kRightLeader);
            
        }
    }
}
