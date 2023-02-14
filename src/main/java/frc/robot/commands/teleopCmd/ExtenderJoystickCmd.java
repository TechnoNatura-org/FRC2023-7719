// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleopCmd;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ExtenderSubsystem;

public class ExtenderJoystickCmd extends CommandBase {
  private final ExtenderSubsystem extenderSubsystem;
  private Supplier<Integer> value;

  /** Creates a new ExtenderJoystickCmd. */
  public ExtenderJoystickCmd(ExtenderSubsystem extenderSubsystem, Supplier<Integer> value) {
    this.extenderSubsystem = extenderSubsystem;
    this.value = value;
    addRequirements(extenderSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double power = -0.05;
    SmartDashboard.putNumber("povButton", value.get());
    if (value.get() == 0) {
      power = 1.0;
    } else if (value.get() == -180) {
      power = -1.0;
    }
    extenderSubsystem.setPower(power);
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
