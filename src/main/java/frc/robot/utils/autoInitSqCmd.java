package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.PIDArmSubsystem;

public class autoInitSqCmd extends SequentialCommandGroup {
    public autoInitSqCmd(PIDArmSubsystem armSubsystem, Command... commands) {
        addCommands(armSubsystem.reset());
        addCommands(armSubsystem.setPosCmd(0));
        addCommands(commands);
    }
}
