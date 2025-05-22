package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.Constants.ElevatorConstants;


public class AutoCoralStation extends SequentialCommandGroup {
    public AutoCoralStation(Elevator elevator, Arm arm, Intake intake, LED led) {
        addCommands(
            //new WaitCommand(0.25),
            new WaitCommand(.2),
            new LEDSolidRed(led),
            new ChangeAngle(arm, ElevatorConstants.kArmTravel),
            new ChangeHeight(elevator, ElevatorConstants.kCorralStation),
            new ChangeAngle(arm, ElevatorConstants.kCorralStationArm),
            new IntakeCurrentStop(intake, 0.15),
            new LEDSolidGreen(led),
            new IntakeSetSpeed(intake, -.15),
            new WaitCommand(.15),
            new IntakeSetSpeed(intake, 0)
            
        );
    }
}