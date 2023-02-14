// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCmd;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PIDArmSubsystem;
import frc.robot.subsystems.PIDElevatorSubsystem;
import frc.robot.subsystems.PIDElevatorSubsystem2;

public class ManipulatorPosCmd extends CommandBase {
  private final PIDElevatorSubsystem elevatorSubsystem;
  private final PIDArmSubsystem armSubsystem;
  private final IntakeSubsystem intakeSubsystem;
  private Supplier<Boolean> IntakeEnabled;
  private Supplier<Boolean> IntakeReverse;
  private int kPosition;
  private double elevatorSetpoint = 0;

  /** Creates a new ManipulatorPosCmd. */
  public ManipulatorPosCmd(PIDElevatorSubsystem elevatorSubsystem, PIDArmSubsystem armSubsystem,
      IntakeSubsystem intakeSubsystem, Supplier<Boolean> IntakeEnabled, Supplier<Boolean> IntakeReverse,
      int kPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.IntakeEnabled = IntakeEnabled;
    this.IntakeReverse = IntakeReverse;
    this.kPosition = kPosition;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (kPosition == ManipulatorConstants.kFrontGroundPosition) {
      armSubsystem.setPos(45);
    }

    if (IntakeEnabled.get() == true) {
      intakeSubsystem.setPower(1);
      elevatorSetpoint = 0.7;
    } else if (IntakeReverse.get() == true) {
      intakeSubsystem.setPower(-1);
      elevatorSetpoint = 0.7;
    } else {
      intakeSubsystem.setPower(0);
    }

    elevatorSubsystem.setPos(elevatorSetpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
