package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.JKAutoProfile;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.autonomousCmd.ArmPositionCmd;
import frc.robot.commands.autonomousCmd.EngageCmd;
import frc.robot.commands.autonomousCmd.ExtenderSetPowerCmd;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.PIDArmSubsystem;
import frc.robot.subsystems.PIDElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.autoInitSqCmd;

class PathPlannerSwerveGenerator {
    public PathPlannerSwerveGenerator() {

    }

}

public final class Autos {

    public static JKAutoProfile posTopLeave(String pathName, SwerveSubsystem driveSubsystem,
            PIDArmSubsystem armSubsystem, ExtenderSubsystem extenderSubsystem,
            PIDElevatorSubsystem pidElevatorSubsystem) {

        PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, AutoConstants.maxVelocityAcceleration);

        CommandBase LeaveCommunity = runPath(driveSubsystem, pathTrajectory);
        SequentialCommandGroup command = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    driveSubsystem.resetOdometry(pathTrajectory.getInitialHolonomicPose());
                }));

        return new JKAutoProfile(armSubsystem, extenderSubsystem, pidElevatorSubsystem, command,
                pathTrajectory.getInitialPose());
    }

    public static JKAutoProfile posTopDock(String pathName, SwerveSubsystem driveSubsystem,
            PIDArmSubsystem armSubsystem, ExtenderSubsystem extenderSubsystem,
            PIDElevatorSubsystem pidElevatorSubsystem) {

        // PID
        SequentialCommandGroup command = new SequentialCommandGroup(
                runPathSeqBase(driveSubsystem, pathName, false));

        return new JKAutoProfile(armSubsystem, extenderSubsystem, pidElevatorSubsystem, command);

        // return new JKAutoProfile();
    }

    // public static JKAutoProfile posTopPick(String pathName) {
    // return new JKAutoProfile();
    // }

    public static JKAutoProfile PIDSTester(String pathName, SwerveSubsystem driveSubsystem,
            PIDArmSubsystem armSubsystem, ExtenderSubsystem extenderSubsystem,
            PIDElevatorSubsystem pidElevatorSubsystem) {

        PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(0.5, 0.5));

        CommandBase LeaveCommunity = runPath(driveSubsystem, pathTrajectory);
        SequentialCommandGroup command = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    driveSubsystem.resetOdometry(pathTrajectory.getInitialHolonomicPose());
                }), LeaveCommunity);

        return new JKAutoProfile(armSubsystem, extenderSubsystem, pidElevatorSubsystem, command,
                pathTrajectory.getInitialPose());
    }

    public static JKAutoProfile posMidLeave(String pathName, PIDArmSubsystem armSubsystem) {
        return new JKAutoProfile();
    }

    public static JKAutoProfile posMidDock(String pathName, PIDArmSubsystem armSubsystem) {
        return new JKAutoProfile();
    }

    public static JKAutoProfile posMidPick(String pathName, PIDArmSubsystem armSubsystem) {
        return new JKAutoProfile();
    }

    public static JKAutoProfile posLeaveLeave(String pathName, PIDArmSubsystem armSubsystem) {
        return new JKAutoProfile();
    }

    public static JKAutoProfile posLeaveDock(String pathName, PIDArmSubsystem armSubsystem) {
        return new JKAutoProfile();
    }

    public static JKAutoProfile posLeavePick(String pathName, PIDArmSubsystem armSubsystem) {
        return new JKAutoProfile();
    }

    public static CommandBase runPath(SwerveSubsystem driveSubsystem, PathPlannerTrajectory pathTrajectory) {
        return new PPSwerveControllerCommand(pathTrajectory, driveSubsystem::getPose2d, DriveConstants.kDriveKinematics,
                AutoConstants.xController, AutoConstants.yController, AutoConstants.thetaController,
                driveSubsystem::setModuleState, false, driveSubsystem);
    }

    public static CommandBase runPathSeqBase(SwerveSubsystem driveSubsystem, String pathName, boolean isSelfCorrect) {

        PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(0.5, 0.5));

        CommandBase PathPlannerControllerCommand = runPath(driveSubsystem, pathTrajectory);

        return new SequentialCommandGroup(new InstantCommand(() -> {
            if (isSelfCorrect == false) {
                driveSubsystem.resetOdometry(pathTrajectory.getInitialHolonomicPose());
            }
        }), PathPlannerControllerCommand);

    }

    public static JKAutoProfile MidEngageTesting(String pathName, SwerveSubsystem driveSubsystem,
            PIDArmSubsystem armSubsystem) {
        PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(0.5, 0.5));

        CommandBase MidEngageTesting = runPath(driveSubsystem, pathTrajectory);

        EngageCmd engageNow = new EngageCmd(driveSubsystem);

        SequentialCommandGroup command = new SequentialCommandGroup(new InstantCommand(() -> {
            driveSubsystem.resetOdometry(pathTrajectory.getInitialHolonomicPose());
        }), MidEngageTesting);

        return new JKAutoProfile(command,
                pathTrajectory.getInitialPose());
    }

    public static JKAutoProfile engageTestingMoving(PIDArmSubsystem armSubsystem, ExtenderSubsystem extenderSubsystem,
            PIDElevatorSubsystem pidElevatorSubsystem, SwerveSubsystem swerveSubsystem) {

        SequentialCommandGroup command = new SequentialCommandGroup(
                new ArmPositionCmd(armSubsystem, 40),
                new ExtenderSetPowerCmd(extenderSubsystem, 1, 1),
                new EngageCmd(swerveSubsystem));

        return new JKAutoProfile(armSubsystem, extenderSubsystem, pidElevatorSubsystem, command);
    }

    public static JKAutoProfile engageTesting(PIDArmSubsystem armSubsystem, ExtenderSubsystem extenderSubsystem,
            PIDElevatorSubsystem pidElevatorSubsystem, SwerveSubsystem swerveSubsystem) {

        SequentialCommandGroup command = new SequentialCommandGroup(

                new EngageCmd(swerveSubsystem));

        return new JKAutoProfile(armSubsystem, extenderSubsystem, pidElevatorSubsystem, command);
    }

    public static JKAutoProfile armTesting(PIDArmSubsystem armSubsystem, ExtenderSubsystem extenderSubsystem,
            PIDElevatorSubsystem pidElevatorSubsystem) {

        SequentialCommandGroup command = new SequentialCommandGroup(
                new ArmPositionCmd(armSubsystem, ManipulatorConstants.kFrontGroundPosition),
                new WaitCommand(5),
                new ArmPositionCmd(armSubsystem, 0));

        return new JKAutoProfile(armSubsystem, extenderSubsystem, pidElevatorSubsystem, command);
    }

    // public static CommandBase swerveCCommand() {
    // // return new;
    // }
    // private static

}
