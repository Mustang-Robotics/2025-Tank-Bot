package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class ShooterSubsystem extends SubsystemBase {
    private final SparkMax m_Shooter = new SparkMax(9, MotorType.kBrushless);
    private double targetSpeed = 0; 
    private RelativeEncoder shooterEncoder = m_Shooter.getEncoder();
    private SparkClosedLoopController shooterClosedLoopController = m_Shooter.getClosedLoopController();
    public ShooterSubsystem(){
        m_Shooter.configure(
            Configs.Shooter.ShooterConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);
            shooterEncoder.setPosition(0);

    }

    public void setSpeed(double speed){
       targetSpeed = speed;

    }

    private void moveToSetpoint() {
    
    shooterClosedLoopController.setReference(
        targetSpeed, ControlType.kMAXMotionVelocityControl, ClosedLoopSlot.kSlot0, 0.0);
  }

  @Override
  public void periodic() {
    moveToSetpoint();
  }

}
