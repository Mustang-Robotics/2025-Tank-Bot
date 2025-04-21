package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDrive extends Command{
    DriveSubsystem m_drive;
    DoubleSupplier m_xSupplier;
    DoubleSupplier m_zSupplier;
  
    public ArcadeDrive(DriveSubsystem drive, DoubleSupplier xSupplier, DoubleSupplier zSupplier){
      m_drive = drive;
      m_xSupplier = xSupplier;
      m_zSupplier = zSupplier;

      addRequirements(m_drive);
    }

    @Override
    public void execute(){
      double x = MathUtil.applyDeadband(m_xSupplier.getAsDouble(), OperatorConstants.kDeadband);
      double z = MathUtil.applyDeadband(m_zSupplier.getAsDouble(), OperatorConstants.kDeadband);

      var speeds = DifferentialDrive.arcadeDriveIK(x, z, true);
      
      m_drive.runOpenLoop(speeds.left * DriveConstants.kMaxSpeedMetersPerSec * DriveConstants.kVoltPerSpeed, speeds.right * DriveConstants.kMaxSpeedMetersPerSec * DriveConstants.kVoltPerSpeed);
    }
}
