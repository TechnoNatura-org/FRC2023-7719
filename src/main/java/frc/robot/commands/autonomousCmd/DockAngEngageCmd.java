package frc.robot.commands.autonomousCmd;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.PIDArmSubsystem;

public class DockAngEngageCmd extends SequentialCommandGroup {
    public DockAngEngageCmd(PIDArmSubsystem armSubsystem, ExtenderSubsystem extenderSubsystem) {
        addCommands(new ArmPositionCmd(armSubsystem, 30), new WaitCommand(1),
                new ExtenderSetPowerCmd(extenderSubsystem, 1, 1));
    }
}
