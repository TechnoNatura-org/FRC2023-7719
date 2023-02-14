package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomousCmd.ArmPositionCmd;
import frc.robot.commands.autonomousCmd.ElevatorHasResetCmd;
import frc.robot.commands.autonomousCmd.ElevatorPositionCmd;
import frc.robot.commands.autonomousCmd.ExtenderSetPowerCmd;
import frc.robot.commands.autonomousCmd.ManipulatorPositionCmd;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.PIDArmSubsystem;
import frc.robot.subsystems.PIDElevatorSubsystem;

public class autoInitSqCmd extends SequentialCommandGroup {
        public autoInitSqCmd(PIDArmSubsystem armSubsystem, ExtenderSubsystem extenderSubsystem,
                        PIDElevatorSubsystem elevatorSubsystem, Command commands) {

                addCommands(Commands.waitUntil(() -> elevatorSubsystem.atHome()), armSubsystem.resetCmd(),
                                new ExtenderSetPowerCmd(extenderSubsystem, -0.05),
                                new ArmPositionCmd(armSubsystem, 30),
                                new WaitCommand(10),
                                new ArmPositionCmd(armSubsystem, -20),
                                commands);
        }
}
