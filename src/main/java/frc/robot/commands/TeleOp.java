package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleOp extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpeedFunction, ySpeedFunction, turningSpeedFuntion;
    private final Supplier<Boolean> fieldOrientedFunction;
	private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;

    public TeleOp(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpeedFunction,
            Supplier<Double> ySpeedFunction, Supplier<Double> turningSpeedFuntion,
            Supplier<Boolean> fieldOrientedFunction) {
        // this.swerveSubsystem =
        this.xSpeedFunction = xSpeedFunction;
        this.ySpeedFunction = ySpeedFunction;
        this.turningSpeedFuntion = turningSpeedFuntion;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.swerveSubsystem = swerveSubsystem;

        this.xLimiter = new SlewRateLimiter(2);
		this.yLimiter = new SlewRateLimiter(2);
		this.turnLimiter = new SlewRateLimiter(2);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        // super.initialize();
    }

    @Override
    public void execute() {
        double xSpeed = xSpeedFunction.get();
        double ySpeed = ySpeedFunction.get();
        double turnSpeed = turningSpeedFuntion.get();

        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turnSpeed = Math.abs(turnSpeed) > OIConstants.kDeadband ? turnSpeed : 0.0;

        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turnSpeed,
                    swerveSubsystem.getRotation2d());
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        }

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleState(moduleStates);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
