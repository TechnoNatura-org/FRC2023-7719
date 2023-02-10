package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDArmSubsystem;
import frc.robot.subsystems.PIDElevatorSubsystem;

public class PIDManipulatorCmd extends CommandBase {
    private PIDElevatorSubsystem pidElevatorSubsystem;
    private PIDArmSubsystem pidArmSubsystem;
    private double elevatorSetpoint = 0.0;
    private double armSetpoint = 0.0;

    public PIDManipulatorCmd(PIDElevatorSubsystem pidElevatorSubsystem, PIDArmSubsystem pidArmSubsystem,
            double elevatorSetpoint, double armSetpoint) {
        this.pidElevatorSubsystem = pidElevatorSubsystem;
        this.pidArmSubsystem = pidArmSubsystem;
        this.elevatorSetpoint = elevatorSetpoint;
        this.armSetpoint = armSetpoint;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        pidElevatorSubsystem.setPos(elevatorSetpoint);
        pidArmSubsystem.setPos(armSetpoint);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
