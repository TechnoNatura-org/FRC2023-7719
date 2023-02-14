package frc.robot.commands.autonomousCmd;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDArmSubsystem;
import frc.robot.subsystems.PIDElevatorSubsystem;

public class ManipulatorPositionCmd extends CommandBase {
    private PIDElevatorSubsystem pidElevatorSubsystem;
    private PIDArmSubsystem pidArmSubsystem;

    private double lifterSetpoint = 0, armSetpoint = 0;

    public ManipulatorPositionCmd(PIDElevatorSubsystem pidElevatorSubsystem, PIDArmSubsystem armSubsystem,
            double armSetpoint, double lifterSetpoint) {
        this.pidElevatorSubsystem = pidElevatorSubsystem;
        this.pidArmSubsystem = armSubsystem;
        this.lifterSetpoint = lifterSetpoint;
        this.armSetpoint = armSetpoint;
        addRequirements(pidElevatorSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        pidElevatorSubsystem.setPos(lifterSetpoint);
        pidArmSubsystem.setPos(armSetpoint);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
