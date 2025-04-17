// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Configs;
import frc.robot.Constants.AutoConfig;

public class DriveSubsystem extends SubsystemBase {
  
  private final SparkMax m_leftLeader =  new SparkMax(31, MotorType.kBrushless);
  private final SparkMax m_leftFollower =  new SparkMax(32, MotorType.kBrushless);
  private final SparkMax m_rightLeader =  new SparkMax(33, MotorType.kBrushless);
  private final SparkMax m_rightFollower =  new SparkMax(34, MotorType.kBrushless);
  private final RelativeEncoder leftEncoder = m_leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = m_rightLeader.getEncoder();
  private final SparkClosedLoopController leftController = m_leftLeader.getClosedLoopController();
  private final SparkClosedLoopController rightController = m_rightLeader.getClosedLoopController();
  
  double wheelRadiusMeters = Units.inchesToMeters(3);
  double trackWidth = .557;
  private final DifferentialDriveKinematics kinematics =new DifferentialDriveKinematics(trackWidth);
  
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  public final Field2d m_pose = new Field2d();
  public Pose2d initialPosition = new Pose2d();
  private final DifferentialDrivePoseEstimator m_odometry = 
    new DifferentialDrivePoseEstimator(kinematics, Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)), 0.0, 0.0, initialPosition);
  private final Vector<N3> qelems = VecBuilder.fill(0.0625, 0.125, .125);
  private final Vector<N2> relems = VecBuilder.fill(.5, 1);
  public Vision vision = new Vision();
  
  public DriveSubsystem() {
    
    m_leftLeader.configure(
      Configs.DriveSystem.leftLeadConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
      
    m_leftFollower.configure(
      Configs.DriveSystem.leftFollowConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
      
    m_rightLeader.configure(
      Configs.DriveSystem.rightLeadConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
      
    m_rightFollower.configure(
      Configs.DriveSystem.rightFollowConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    leftEncoder.setPosition(0);
    rightEncoder.setPosition(0);

    AutoBuilder.configure(
    this::getPose, // Robot pose supplier
    this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
    this::getRobotRelaviteSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
    (ChassisSpeeds speeds) -> runClosedLoop(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
    new PPLTVController(qelems, relems, 0.2, 3.2),
    AutoConfig.config,
    () -> {
      var alliance = DriverStation.getAlliance();
      if(alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
    },
    this // Reference to this subsystem to set requirements
  );
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("angle", m_gyro.getAngle(IMUAxis.kZ));
    m_pose.setRobotPose(m_odometry.getEstimatedPosition());
    SmartDashboard.putNumber("X", m_pose.getRobotPose().getMeasureX().baseUnitMagnitude());
    SmartDashboard.putNumber("Y", m_pose.getRobotPose().getMeasureY().baseUnitMagnitude());
    SmartDashboard.putNumber("Rotation", m_pose.getRobotPose().getRotation().getDegrees());
    SmartDashboard.putData("pose", m_pose);
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        getLeftPositionMeters(),
        getRightPositionMeters());

    var visionBottomEst = vision.getBottomEstimatedGlobalPose();
    visionBottomEst.ifPresent(
      est -> {
        var estStdDevs = vision.getEstimationStdDevs();
        m_odometry.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
    });
  }

  public void setVoltage(double leftVolts, double rightVolts) {
    m_leftLeader.setVoltage(leftVolts);
    m_rightLeader.setVoltage(rightVolts);
  }

  public void setVelocity(double leftRadPerSec, double rightRadPerSec) {
    leftController.setReference(leftRadPerSec, ControlType.kVelocity);
    rightController.setReference(rightRadPerSec, ControlType.kVelocity);
  }

  public void runClosedLoop(ChassisSpeeds speeds) {
    var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    runClosedLoop(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  public void runClosedLoop(double leftMetersPerSec, double rightMetersPerSec) {
    double leftRadPerSec = leftMetersPerSec / wheelRadiusMeters;
    double rightRadPerSec = rightMetersPerSec / wheelRadiusMeters;

    setVelocity(leftRadPerSec, rightRadPerSec);
  }
  
  public void runOpenLoop(double leftVolts, double rightVolts) {
    setVoltage(leftVolts, rightVolts);
  }

  public void stop() {
    runOpenLoop(0.0, 0.0);
  }

  public double getLeftPositionMeters() {
    return leftEncoder.getPosition() * wheelRadiusMeters;
  }

  public double getRightPositionMeters() {
    return rightEncoder.getPosition() * wheelRadiusMeters;
  }

  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        getLeftPositionMeters(),
        getRightPositionMeters(),
        pose);
  }

  public double getLeftVelocityMPS() {
    return leftEncoder.getVelocity() * wheelRadiusMeters;
  }

  public double getRightVelocityMPS() {
    return rightEncoder.getVelocity() * wheelRadiusMeters;
  }

  public ChassisSpeeds getRobotRelaviteSpeeds() {
    DifferentialDriveWheelSpeeds speeds = new DifferentialDriveWheelSpeeds(getLeftVelocityMPS(), getRightVelocityMPS());
    return kinematics.toChassisSpeeds(speeds);
  }

}
