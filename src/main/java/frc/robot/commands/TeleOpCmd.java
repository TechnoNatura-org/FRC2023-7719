package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.PIDElevatorSubsystem2;
import frc.robot.subsystems.SwerveSubsystem;

public class TeleOpCmd extends CommandBase {
    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpeedFunction, ySpeedFunction, turningSpeedFuntion;
    private final Supplier<Double> fullSpeed;

    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(2), yLimiter = new SlewRateLimiter(2),
            turnLimiter = new SlewRateLimiter(
                    2);

    public TeleOpCmd(SwerveSubsystem swerveSubsystem, Supplier<Double> xSpeedFunction,
            Supplier<Double> ySpeedFunction, Supplier<Double> turningSpeedFuntion,
            Supplier<Boolean> fieldOrientedFunction, Supplier<Double> fullSpeed) {
        // this.swerveSubsystem =

        // this.liftSupplier = liftSupplier;
        this.fullSpeed = fullSpeed;
        // this.linkageSupplier = linkageSupplier;

        this.xSpeedFunction = xSpeedFunction;
        this.ySpeedFunction = ySpeedFunction;
        this.turningSpeedFuntion = turningSpeedFuntion;
        this.fieldOrientedFunction = fieldOrientedFunction;
        this.swerveSubsystem = swerveSubsystem;

        // this.xLimiter = new SlewRateLimiter(2);
        // this.yLimiter = new SlewRateLimiter(2);
        // this.turnLimiter = new SlewRateLimiter(2);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        // super.initialize();
    }

    @Override
    public void execute() {
        // double lifterSpeed = xSpeedFunction.get();
        // this.lifterSubsystem

        double xSpeed = xSpeedFunction.get();
        double ySpeed = ySpeedFunction.get();
        double turnSpeed = turningSpeedFuntion.get();

        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;

        turnSpeed = Math.abs(turnSpeed) > OIConstants.kDeadband ? turnSpeed : 0.0;

        if (fullSpeed.get() < 0.3) {
            xSpeed /= 4;
            ySpeed /= 4;
            turnSpeed /= 4;

        }

        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turnSpeed = turnLimiter.calculate(turnSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

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
