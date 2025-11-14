package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;


public class SetShooterSpeed extends Command {
    private final ShooterSubsystem m_shooter;
    private double m_setpoint;

    public SetShooterSpeed(ShooterSubsystem shooter, double setpoint) {
        m_shooter = shooter;
        m_setpoint = setpoint;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.setSpeed(m_setpoint);
    }

    @Override
    public boolean isFinished() {
        return MathUtil.isNear(m_setpoint,m_shooter.shooterEncoder.getVelocity(),200);
    }
}