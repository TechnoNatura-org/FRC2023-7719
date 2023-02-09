// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private final Talon m_intakeMotor = new Talon(1);
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double value){
    m_intakeMotor.set(value);
  }

  public void Enable(){
    m_intakeMotor.set(.5);
  }

  public void Reverse(){
    m_intakeMotor.set(-1.0);
  }
}
