package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.PIDArmSubsystem;
import frc.robot.subsystems.PIDElevatorSubsystem;
import frc.robot.utils.autoInitSqCmd;

public class JKAutoProfile {
    private CommandBase sequence;
    private Pose2d initialPose;

    public JKAutoProfile() {
        sequence = new SequentialCommandGroup();
        initialPose = new Pose2d();
    }

    public JKAutoProfile(CommandBase commandSequence, Pose2d initialPose2d) {
        sequence = commandSequence;
        initialPose = new Pose2d();
    }

    public JKAutoProfile(PIDArmSubsystem armSubsystem, ExtenderSubsystem extenderSubsystem,
            PIDElevatorSubsystem elevatorSubsystem, CommandBase commandSequence) {
        sequence = new autoInitSqCmd(armSubsystem, extenderSubsystem, elevatorSubsystem, commandSequence);
        initialPose = new Pose2d();
    }

    public JKAutoProfile(PIDArmSubsystem armSubsystem, ExtenderSubsystem extenderSubsystem,
            PIDElevatorSubsystem elevatorSubsystem, CommandBase commandSequence, Pose2d intialPose2d) {
        sequence = new autoInitSqCmd(armSubsystem, extenderSubsystem, elevatorSubsystem, commandSequence);
        initialPose = intialPose2d;
    }

    public CommandBase getAutoCommand() {
        return sequence;
    }

    public Pose2d getStartingPose2d() {
        return initialPose;
    }

}
