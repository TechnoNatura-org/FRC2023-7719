// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PIDElevatorSubsystem;

public class ElevatorJoystickCmd extends CommandBase {
  private final PIDElevatorSubsystem elevatorSubsystem;
  private Supplier<Double> joystickAxis;
  private double setpoint = 0;

  /** Creates a new ElevatorJoystickCmd. */
  public ElevatorJoystickCmd(PIDElevatorSubsystem elevatorSubsystem, Supplier<Double> joystickAxis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorSubsystem = elevatorSubsystem;
    this.joystickAxis = joystickAxis;
    addRequirements(elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double value = Math.abs(joystickAxis.get()) > 0.1 ? joystickAxis.get() : 0;
    setpoint += value * 0.01;
    setpoint = MathUtil.clamp(setpoint, -0.05, 0.8);
    elevatorSubsystem.setPos(setpoint);
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
