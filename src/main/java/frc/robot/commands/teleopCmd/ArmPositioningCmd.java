package frc.robot.commands.teleopCmd;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.PIDArmSubsystem;

public class ArmPositioningCmd extends CommandBase {
    private PIDArmSubsystem armSubsystem;
    private Supplier<Double> leftX;
    private SlewRateLimiter limiter = new SlewRateLimiter(2);
    // private PIDController m_armPID = new PIDController(2.5, 0.2, 0.5);
    private double setpoint = 0;

    public ArmPositioningCmd(PIDArmSubsystem armSubsystem, Supplier<Double> leftX) {

        this.leftX = leftX;

        this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);

    }

    public void teleOpInit() {
        setpoint = armSubsystem.getPosition();
    }

    @Override
    public void execute() {
        SmartDashboard.putString("STATUS WOYYY", Constants.status);
        // SmartDashboard.putNumber("left X", leftX.get());
        double leftX = this.leftX.get();

        SmartDashboard.putNumber("leftX ps4", this.leftX.get());

        if (Constants.status == "teleop") {
            // SmartDashboard.putNumber("leftY ps4", this.leftY.get());
            // SmartDashboard.putNumber("rightX ps4", rightX.get());
            // SmartDashboard.putNumber("rightY ps4", rightY.get());

            double xSpeed = leftX;

            if (xSpeed > 0.05 || xSpeed < 0.05) {
                // armSubsystem.setSpeed(xSpeed);
                setpoint += leftX;
            }

            if (armSubsystem.getPosition() != setpoint) {
                armSubsystem.setSetpoint(setpoint);
            }
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
