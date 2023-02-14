package frc.robot.commands.autonomousCmd;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtenderSubsystem;

public class ExtenderSetPowerCmd extends CommandBase {

    private boolean isDuration = false;
    private ExtenderSubsystem extenderSubsystem;
    private double duration = 0;
    private double initTime = 0;
    private double power = 0;
    private boolean isFinished = false;

    public ExtenderSetPowerCmd(ExtenderSubsystem extenderSubsystem, double power, double duration) {
        isDuration = true;
        this.duration = duration;
        this.isDuration = true;
        this.power = power;
        this.extenderSubsystem = extenderSubsystem;
        addRequirements(extenderSubsystem);
    }

    public ExtenderSetPowerCmd(ExtenderSubsystem extenderSubsystem, double power) {
        isDuration = true;
        this.power = power;
        this.extenderSubsystem = extenderSubsystem;
        addRequirements(extenderSubsystem);
    }

    @Override
    public void initialize() {
        this.initTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {

        if (this.isDuration == true) {
            double timeNow = Timer.getFPGATimestamp();
            if (timeNow - initTime < duration) {
                extenderSubsystem.setPower(this.power);
            } else {
                extenderSubsystem.setPower(0);
                isFinished = true;
            }
        } else {
            extenderSubsystem.setPower(power);
            isFinished = true;
        }

    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return isFinished;
    }

}
