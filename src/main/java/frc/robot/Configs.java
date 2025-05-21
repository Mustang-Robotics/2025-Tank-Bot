/*Configs are used to set motor controller options
See aditional comments below for what each config is doing*/

//This section is all of the classes that are being use in this section of the code.
package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.DriveConstants;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public final class Configs {
    //All motors contained within the the DriveSystem.
    //See comment below for where any new subsystem motors would go using the same format
    public static final class DriveSystem {
        //All motor types included in drivesystem
        //If multiple motors use the same config setup you only need to make one set for it.
        public static final SparkMaxConfig leftLeadConfig = new SparkMaxConfig();
        public static final SparkMaxConfig leftFollowConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rightLeadConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rightFollowConfig = new SparkMaxConfig();

        static {

            leftLeadConfig //everything below here is modifying the leftLead
                .idleMode(IdleMode.kBrake) //idleMode controlls if the motor is free spinning or not if there is no output
                .smartCurrentLimit(DriveConstants.kStallCurrent) //Sets the max amount of current the motor can get. This value is in Constants
                .voltageCompensation(12); //Normalize the battery voltage to 12. This should almost always be 12.
            
            leftLeadConfig //the following two leftLeadConfig sections could've been included in the first one this is just visually easier
                .encoder //modifying encoder parameters below this
                .positionConversionFactor(2*Math.PI / DriveConstants.kDrivingMotorReduction) //By default 1 rotation of the motor equals 1 for the encoder. This converts it to angle of wheel rotation
                .velocityConversionFactor(2*Math.PI / 60 / DriveConstants.kDrivingMotorReduction) //Same as above but for the velocity of rotation
                .uvwMeasurementPeriod(10) //Controls how frequently measurements are updated. Leave here unless there is a good reason to change.
                .uvwAverageDepth(2); //Controls how accurate the measurements are. Leave here unless there is a good reason to change.

            leftLeadConfig
                .closedLoop //closedLoop feedback relates to changing the values of the motors based on sensor values. If the motor is not turning the correct amount it will adjust based on these valuse.
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder) //declares which sensor is being used to controll the feedback loop
                .pid(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD) //go to this website for more info https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/introduction-to-pid.html
                .velocityFF(1/DriveConstants.kDriveWheelFreeSpeed) //go to this website for more info https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html#feedforward-control-in-wpilib
                .outputRange(-1, 1); //minimum and maximum speeds of the motor. 1 corresponds to full speed.

            leftFollowConfig
                .idleMode(IdleMode.kBrake) //likely not needed because it is following the Lead
                .smartCurrentLimit(DriveConstants.kStallCurrent) //likely not needed because it is following the Lead
                .voltageCompensation(12) //likely not needed because it is following the Lead
                .follow(DriveConstants.kLeftLeader); //tells which motor to follow. This means it will have all of the same behaviors as the motor it is following

            rightLeadConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(DriveConstants.kStallCurrent)
                .voltageCompensation(12)
                .inverted(true); //flips the direction of forward. Motor will spin the opposite direction of what we tell it.
            
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
                .velocityFF(1/DriveConstants.kDriveWheelFreeSpeed)
                .outputRange(-1, 1);

            rightFollowConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(DriveConstants.kStallCurrent)
                .voltageCompensation(12)
                .follow(DriveConstants.kRightLeader);
            
        }
    }

    //Add new subsystems here. Use formatting style from above
}
