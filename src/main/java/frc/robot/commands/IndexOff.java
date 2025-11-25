package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;


public class IndexOff extends Command {
    private final ShooterSubsystem m_shooter;


    public IndexOff(ShooterSubsystem shooter) {
        m_shooter = shooter;
        addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
        m_shooter.indexOff();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
