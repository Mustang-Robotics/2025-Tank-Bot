package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;


public class IndexOn extends Command {
    private final ShooterSubsystem m_shooter;


    public IndexOn(ShooterSubsystem shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.index();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}