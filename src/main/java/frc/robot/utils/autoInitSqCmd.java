package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class autoInitSqCmd extends SequentialCommandGroup {
    public autoInitSqCmd(Command... commands) {

        addCommands(commands);
    }
}
