package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class JKAutoProfile {
    private CommandBase sequence;
    private Pose2d initialPose;

    public JKAutoProfile() {
        sequence = new SequentialCommandGroup();
        initialPose = new Pose2d();
    }

    public JKAutoProfile(CommandBase commandSequence, Pose2d intialPose2d) {
        sequence = commandSequence;
        initialPose = intialPose2d;
    }

    public CommandBase getAutoCommand() {
        return sequence;
    }

    public Pose2d getStartingPose2d() {
        return initialPose;
    }

}
