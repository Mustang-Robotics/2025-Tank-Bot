package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDrive extends Command{
    DriveSubsystem m_drive;
    private static final double DEADBAND = 0.1;
    DoubleSupplier m_xSupplier;
    DoubleSupplier m_zSupplier;
    double maxSpeedMetersPerSec = 3.2;
  
    public ArcadeDrive(DriveSubsystem drive, DoubleSupplier xSupplier, DoubleSupplier zSupplier)){
      m_drive = drive;
      m_xSupplier = xSupplier;
      m_zSupplier = zSupplier;

      addRequirements(m_drive);
    }

    @Override
    public void execute(){
      double x = MathUtil.applyDeadband(m_xSupplier.getAsDouble(), DEADBAND);
      double z = MathUtil.applyDeadband(m_zSupplier.getAsDouble(), DEADBAND);

      var speeds = DifferentialDrive.arcadeDriveIK(x, z, true);
      
      m_drive.runOpenLoop(speeds.left * maxSpeedMetersPerSec, speeds.right * maxSpeedMetersPerSec);
    }
}
