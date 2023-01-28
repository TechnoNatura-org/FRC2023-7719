package frc.robot.commands;

import java.nio.file.Path;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.JKAutoProfile;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

class PathPlannerSwerveGenerator {
    public PathPlannerSwerveGenerator() {
        
    }

}

public final class Autos {

    public static JKAutoProfile posTopLeave(String pathName, SwerveSubsystem driveSubsystem) {

        PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, AutoConstants.maxVelocityAcceleration);

        CommandBase LeaveCommunity = runPath(driveSubsystem, pathTrajectory);
        SequentialCommandGroup command = new SequentialCommandGroup(new InstantCommand(() -> {
            driveSubsystem.resetOdometry(pathTrajectory.getInitialHolonomicPose());
        }), LeaveCommunity);

        return new JKAutoProfile(command, pathTrajectory.getInitialPose());
    }  
    public static JKAutoProfile PIDSTester(String pathName, SwerveSubsystem driveSubsystem) {

        PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, new PathConstraints(0.5,0.5));

        CommandBase LeaveCommunity = runPath(driveSubsystem, pathTrajectory);
        SequentialCommandGroup command = new SequentialCommandGroup(new InstantCommand(() -> {
            driveSubsystem.resetOdometry(pathTrajectory.getInitialHolonomicPose());
        }), LeaveCommunity);

        return new JKAutoProfile(command, pathTrajectory.getInitialPose());
    }  

    public static JKAutoProfile posTopDock(String pathName,SwerveSubsystem driveSubsystem) {
        PathPlannerTrajectory pathTrajectory = PathPlanner.loadPath(pathName, AutoConstants.maxVelocityAcceleration);

        CommandBase LeaveCommunity = runPath(driveSubsystem, pathTrajectory);
        // PID
        SequentialCommandGroup command = new SequentialCommandGroup(new InstantCommand(() -> {
            driveSubsystem.resetOdometry(pathTrajectory.getInitialHolonomicPose());
        }), LeaveCommunity);

        return new JKAutoProfile(command, pathTrajectory.getInitialPose());
    
        // return new JKAutoProfile();
    } 
    public static JKAutoProfile posTopPick(String pathName) {
        return new JKAutoProfile();
    }
    
    public static JKAutoProfile posMidLeave(String pathName) {
        return new JKAutoProfile();
    }
    public static JKAutoProfile posMidDock(String pathName) {
        return new JKAutoProfile();
    }
    public static JKAutoProfile posMidPick(String pathName) {
        return new JKAutoProfile();
    }

    public static JKAutoProfile posLeaveLeave(String pathName) {
        return new JKAutoProfile();
    }
    public static JKAutoProfile posLeaveDock(String pathName) {
        return new JKAutoProfile();
    }
    public static JKAutoProfile posLeavePick(String pathName) {
        return new JKAutoProfile();
    }

    public static CommandBase runPath(SwerveSubsystem driveSubsystem, PathPlannerTrajectory pathTrajectory) {
        return new PPSwerveControllerCommand(pathTrajectory, driveSubsystem::getPose2d, DriveConstants.kDriveKinematics, AutoConstants.xController, AutoConstants.yController, AutoConstants.thetaController, driveSubsystem::setModuleState,false, driveSubsystem);
    }
    // private static 
    

    

}
