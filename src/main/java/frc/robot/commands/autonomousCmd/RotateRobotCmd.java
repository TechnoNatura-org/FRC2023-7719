package frc.robot.commands.autonomousCmd;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class RotateRobotCmd extends SwerveControllerCommand {

    public RotateRobotCmd(SwerveSubsystem swerveSubsystem, double setpoint,
            ProfiledPIDController profiledThetaController) {

        super(
                TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(0, 0, new Rotation2d(setpoint))),
                        DriveConstants.trajectoryConfig),
                swerveSubsystem::getPose2d, DriveConstants.kDriveKinematics,
                AutoConstants.xController, AutoConstants.yController, profiledThetaController,
                swerveSubsystem::setModuleState, swerveSubsystem);
        // ProfiledPIDController profiledThetaController = new ProfiledPIDController(
        // AutoConstants.kPThetaController, 0, 0,
        // AutoConstants.kThetaControllerConstraints);
        // profiledThetaController.enableContinuousInput(-Math.PI, Math.PI);

    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

}
