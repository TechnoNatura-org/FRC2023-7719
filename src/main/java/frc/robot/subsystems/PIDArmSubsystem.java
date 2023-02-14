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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

    boolean switchVal = homeSwitch.get();
    if (switchVal == false && hasReset == false) {
      m_armEncoder.reset();
      hasReset = true;

      // kArmSetpoint = m_armEncoder.getDistance();
    }

    SmartDashboard.putBoolean("home switch", switchVal);

    // This method will be called once per scheduler run
    SmartDashboard.putNumber("armEncoder", m_armEncoder.getDistance()/* /(7*188) */);
    SmartDashboard.putNumber("armSetpoint", kArmSetpoint);
    SmartDashboard.putBoolean("has reset arm", hasReset);

    // if (Constants.status != "teleop") {
    this.runPID(kArmSetpoint);
    // } else {
    // return;
    // }

    output = -1 * m_armRateLimiter.calculate(output);
    m_armMotor1.set(ControlMode.PercentOutput, output);
    m_armMotor2.set(ControlMode.PercentOutput, output);
  }

  public void runPID(double kArmSetpoint) {
    if (kArmSetpoint >= 0 && m_armEncoder.getDistance() >= -20) {
      output = MathUtil.clamp(
          (m_armPID.calculate(Math.toRadians(m_armEncoder.getDistance()), Math.toRadians(kArmSetpoint))), -0.1, 1.0);
    } else if (kArmSetpoint <= 0 && m_armEncoder.getDistance() < 10) {
      output = MathUtil.clamp(
          (m_armPID.calculate(Math.toRadians(m_armEncoder.getDistance()), Math.toRadians(kArmSetpoint))), -1.0, 0.1);
    }
  }

  public void setPos(double setpoint) {

    if (setpoint > 130) {
      setpoint = 120;
    }

    this.kArmSetpoint = setpoint;
  }

  public void setSpeed(double speed) {
    m_armMotor1.set(ControlMode.PercentOutput, output);
    m_armMotor2.set(ControlMode.PercentOutput, output);
  }

  public CommandBase setPosCmd(double setPoint) {

    return runOnce(() -> {
      System.out.println("change setpoint EE");

      this.kArmSetpoint = setPoint;

      System.out.println("change setpoint");
    });
  }

  public CommandBase resetCmd() {

    return runOnce(() -> {
      hasReset = false;
      m_armEncoder.reset();

      System.out.println("REset");
      Commands.print("Reset");
    });
  }

  public void setSetpoint(double setpoint) {
    this.kArmSetpoint = setpoint;
  }

  public double getSetpoint() {
    return this.kArmSetpoint;
  }

  public void reset() {

    hasReset = false;
    m_armEncoder.reset();

  }

  public double getPosition() {
    return m_armEncoder.getDistance();
  }

  public void resetEncoder() {
    m_armEncoder.reset();
  }
}
