package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class Configs {
    public static final class DriveSystem {
        public static final SparkMaxConfig leftLeadConfig = new SparkMaxConfig();
        public static final SparkMaxConfig leftFollowConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rightLeadConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rightFollowConfig = new SparkMaxConfig();

        static {
            leftLeadConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .voltageCompensation(12);
            
            leftFollowConfig 
                .encoder
                .positionConversionFactor(2*Math.PI / 8.46)
                .velocityConversionFactor(2*Math.PI / 60 / 8.46)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

                leftLeadConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .voltageCompensation(12)
                .follow(31);
            
            leftFollowConfig 
                .encoder
                .positionConversionFactor(2*Math.PI / 8.46)
                .velocityConversionFactor(2*Math.PI / 60 / 8.46)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2);

            rightLeadConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .voltageCompensation(12)
                .inverted(true);
            
            rightLeadConfig 
                .encoder
                .positionConversionFactor(2*Math.PI / 8.46)
                .velocityConversionFactor(2*Math.PI / 60 / 8.46)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2)
                .inverted(true);

            rightFollowConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30)
                .voltageCompensation(12)
                .inverted(true)
                .follow(33);
            
            rightFollowConfig 
                .encoder
                .positionConversionFactor(2*Math.PI / 8.46)
                .velocityConversionFactor(2*Math.PI / 60 / 8.46)
                .uvwMeasurementPeriod(10)
                .uvwAverageDepth(2)
                .inverted(true);
        }
    }
}
