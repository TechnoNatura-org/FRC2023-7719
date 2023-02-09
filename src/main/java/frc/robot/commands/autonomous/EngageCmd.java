package frc.robot.commands.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class EngageCmd extends PIDCommand {
    // private SwerveSubsystem swerveSubsystem;
    public EngageCmd(SwerveSubsystem swerveSubsystem) {
        super(
                new PIDController(AutoConstants.kPXController, 0, AutoConstants.kDXController),
                swerveSubsystem::getPitch,
                0,
                output -> swerveSubsystem.setOutput(output),
                swerveSubsystem);
    }

    @Override
    public void execute() {
        // TODO Auto-generated method stub
        super.execute();
    }

    // fini
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return getController().atSetpoint();
    }

}
