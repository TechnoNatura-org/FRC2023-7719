package frc.robot.commands.autonomousCmd;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDElevatorSubsystem;

public class ElevatorPositionCmd extends CommandBase {
    private PIDElevatorSubsystem pidElevatorSubsystem;
    private double setpoint = 0;

    public ElevatorPositionCmd(PIDElevatorSubsystem pidElevatorSubsystem, double setpoint) {
        this.pidElevatorSubsystem = pidElevatorSubsystem;
        this.setpoint = setpoint;
        addRequirements(pidElevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        pidElevatorSubsystem.setPos(setpoint);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
