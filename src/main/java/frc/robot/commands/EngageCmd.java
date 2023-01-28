package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class EngageCmd extends PIDCommand {
    // private SwerveSubsystem swerveSubsystem;
    public EngageCmd(SwerveSubsystem swerveSubsystem) {
        super(
            new PIDController(0.00001, 0, 0.000009),
            swerveSubsystem::getPitch,
            0,
            output -> swerveSubsystem.setOutput(output),
            swerveSubsystem
        );
        // this.swerveSubsystem = swerveSubsystem;
        
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
