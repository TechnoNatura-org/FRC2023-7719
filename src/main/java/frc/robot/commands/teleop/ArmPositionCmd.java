// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.subsystems.PIDArmSubsystem;

public class ArmPositionCmd extends CommandBase {
  private final PIDArmSubsystem armSubsystem;
  private int kPosition;
  private Supplier<Double> joystickOffset;

  /** Creates a new ArmPositionCmd. */
  public ArmPositionCmd(PIDArmSubsystem armSubsystem, int kPosition, Supplier<Double> joystickOffset) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.kPosition = kPosition;
    this.joystickOffset = joystickOffset;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double Offset = Math.abs(joystickOffset.get()) > 0.1 ? joystickOffset.get() : 0;
    Offset = Offset * 20;

    if (kPosition == ManipulatorConstants.kFrontGroundPosition) {
      armSubsystem.setPos(30 + Offset);
    } else if (kPosition == ManipulatorConstants.kTopNodePosition) {
      armSubsystem.setPos(110 + Offset);
    } else if (kPosition == ManipulatorConstants.kRearGroundPosition) {
      armSubsystem.setPos(-60 + Offset);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setPos(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
