// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class DriveSubsystem extends SubsystemBase {
  
  private final SparkMax m_leftLeader =  new SparkMax(31, MotorType.kBrushless);
  private final SparkMax m_leftFollower =  new SparkMax(32, MotorType.kBrushless);
  private final SparkMax m_rightLeader =  new SparkMax(33, MotorType.kBrushless);
  private final SparkMax m_rightFollower =  new SparkMax(34, MotorType.kBrushless);
  private final RelativeEncoder leftEncoder = m_leftLeader.getEncoder();
  private final RelativeEncoder rightEncoder = m_rightLeader.getEncoder();

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
    
  }

  public void runOpenLoop(double leftVolts, double rightVolts) {
    m_leftLeader.setVoltage(leftVolts);
    m_rightLeader.setVoltage(rightVolts);
  }
}
