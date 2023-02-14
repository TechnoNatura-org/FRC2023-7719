package frc.robot.commands.autonomousCmd;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.PIDArmSubsystem;
import frc.robot.subsystems.PIDElevatorSubsystem;

public class ArmPositionCmd extends CommandBase {
    private final PIDArmSubsystem armSubsystem;
    // private final PIDElevatorSubsystem pidElevatorSubsystem;
    private int kPosition;
    // private boolean hasSetPosition = false;

    /** Creates a new ArmPositionCmd. */
    public ArmPositionCmd(PIDArmSubsystem armSubsystem, int kPosition) {
        // Use addRequirements() here to declare subsystem dependencies.
        this.armSubsystem = armSubsystem;
        // this.pidElevatorSubsystem = pidElevatorSubsystem;
        this.kPosition = kPosition;

        addRequirements(armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // if (hasSetPosition == false) {
        // if (this.pidElevatorSubsystem.atHome() && this.hasSetPosition == false) {
        if (kPosition == ManipulatorConstants.kFrontGroundPosition) {
            armSubsystem.setPos(30);
            return;
        } else if (kPosition == ManipulatorConstants.kTopNodePosition) {
            armSubsystem.setPos(110);
            return;
        } else if (kPosition == ManipulatorConstants.kRearGroundPosition) {
            armSubsystem.setPos(-60);
            return;
        }

        armSubsystem.setPos(kPosition);
        // }
        // hasSetPosition = true;
        // }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
        // return this.pidElevatorSubsystem.atHome() && hasSetPosition;
    }
}
