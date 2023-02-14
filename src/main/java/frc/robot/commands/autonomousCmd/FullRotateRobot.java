package frc.robot.commands.autonomousCmd;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class FullRotateRobot {
    private ProfiledPIDController profiledThetaController;
    private SwerveSubsystem swerveSubsystem;
    private double setPoint;

    public FullRotateRobot(SwerveSubsystem swerveSubsystem, double setpoint) {
        this.swerveSubsystem = swerveSubsystem;
        this.profiledThetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0,
                AutoConstants.kThetaControllerConstraints);
        profiledThetaController.enableContinuousInput(-Math.PI, Math.PI);

    }

    public SwerveControllerCommand run() {
        return new RotateRobotCmd(swerveSubsystem, setPoint, profiledThetaController);
    }
}
