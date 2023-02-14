package frc.robot.commands.autonomousCmd;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtenderSubsystem;

public class RetractExtenderCmd extends CommandBase {
    private ExtenderSubsystem extenderSubsystem;

    public RetractExtenderCmd(ExtenderSubsystem extenderSubsystem) {
        this.extenderSubsystem = extenderSubsystem;
        addRequirements(extenderSubsystem);
    }

    @Override
    public void execute() {
        this.extenderSubsystem.setPower(-1);
    }

    @Override
    public void end(boolean interrupted) {
        this.extenderSubsystem.setPower(0);

    }

    @Override
    public boolean isFinished() {
        return extenderSubsystem.atHome();
    }
}
