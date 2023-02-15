package frc.robot.commands.autonomousCmd;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class EngageCmd extends CommandBase {

    private PIDController pidController = new PIDController(AutoConstants.kPXController, 0,
            AutoConstants.kDXController);
    private SwerveSubsystem swerveSubsystem;
    private boolean hasElevated = false;

    public EngageCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        pidController.setTolerance(1);
    }

    @Override
    public void execute() {
        double pitch = Math.abs(swerveSubsystem.getPitch());
        double speed = 0;
        // ChassisSpeeds chassisSpeeds;

        if (pitch > 5) {
            hasElevated = true;

            speed = MathUtil.clamp(pidController.calculate(pitch, 0), -0.2, 0.2);

        } else {
            speed = 0.25;
        }
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, speed, 0);
        // chassisSpeeds
        swerveSubsystem.setModuleState(Constants.DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));

    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }
}
