package frc.robot.commands.teleopCmd;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtenderSubsystem;

public class ExtenderPositioningJoystickCmd extends CommandBase {
    private Supplier<Double> rightXJoystick;
    private ExtenderSubsystem extenderSubsystem;
    private SlewRateLimiter rateLimiter = new SlewRateLimiter(2);

    public ExtenderPositioningJoystickCmd(ExtenderSubsystem extenderSubsystem, Supplier<Double> rightX) {
        this.extenderSubsystem = extenderSubsystem;
        this.rightXJoystick = rightX;

        addRequirements(extenderSubsystem);
    }

    @Override
    public void execute() {
        double xSpeed = rateLimiter.calculate(rightXJoystick.get());

        SmartDashboard.putNumber("rightX ps4", xSpeed);
        extenderSubsystem.setPower(xSpeed);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
