// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtenderSubsystem extends SubsystemBase {
  private final VictorSP m_extenderMotor = new VictorSP(0);
  private final DigitalInput m_MagneticSwitch = new DigitalInput(3);
  private boolean atHome;

  /** Creates a new ExtenderSubsystem. */
  public ExtenderSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    atHome = m_MagneticSwitch.get();
    SmartDashboard.putBoolean("externalHomePosition", atHome);

  }

  public void setPower(double power) {
    m_extenderMotor.set(power);
  }

  public boolean atHome() {
    return !atHome;
  }
}
