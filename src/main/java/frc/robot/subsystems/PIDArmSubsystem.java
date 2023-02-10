// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDArmSubsystem extends SubsystemBase {
  private TalonSRX m_armMotor1 = new TalonSRX(15);
  private TalonSRX m_armMotor2 = new TalonSRX(16);
  private Encoder m_armEncoder = new Encoder(1, 2);

  private DigitalInput homeSwitch = new DigitalInput(4);
  private boolean hasReset = false;

  private PIDController m_armPID = new PIDController(2.5, 0.2, 0.5);
  private SlewRateLimiter m_armRateLimiter = new SlewRateLimiter(1);
  private double kArmSetpoint = 0;
  private double output;

  /** Creates a new LinkageSubsystem. */
  public PIDArmSubsystem() {
    m_armMotor2.setInverted(true);
    m_armMotor1.setInverted(false);
    m_armEncoder.setDistancePerPulse(360. / 1316.);
    m_armEncoder.reset();
  }

  @Override
  public void periodic() {

    if (homeSwitch.get() == false && hasReset == false) {
      m_armEncoder.reset();
      hasReset = true;

      kArmSetpoint = m_armEncoder.getDistance();
    }

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("armEncoder", m_armEncoder.getDistance()/* /(7*188) */);
    SmartDashboard.putNumber("armSetpoint", kArmSetpoint);

    if (kArmSetpoint >= 0 && m_armEncoder.getDistance() >= -20) {
      output = MathUtil.clamp(
          (m_armPID.calculate(Math.toRadians(m_armEncoder.getDistance()), Math.toRadians(kArmSetpoint))), -0.1, 1.0);
    } else if (kArmSetpoint <= 0 && m_armEncoder.getDistance() < 10) {
      output = MathUtil.clamp(
          (m_armPID.calculate(Math.toRadians(m_armEncoder.getDistance()), Math.toRadians(kArmSetpoint))), -1.0, 0.1);
    }

    output = -1 * m_armRateLimiter.calculate(output);
    m_armMotor1.set(ControlMode.PercentOutput, output);
    m_armMotor2.set(ControlMode.PercentOutput, output);
  }

  public void setPos(double setpoint) {
    this.kArmSetpoint = setpoint;
  }

  public CommandBase setPosCmd(double setPoint) {
    return runOnce(() -> {
      this.kArmSetpoint = setPoint;
    });
  }

  public CommandBase reset() {
    return runOnce(() -> {
      hasReset = false;
    });
  }

  public double getPosition() {
    return m_armEncoder.getDistance();
  }
}
