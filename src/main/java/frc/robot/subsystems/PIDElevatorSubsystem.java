// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LifterConstants;

public class PIDElevatorSubsystem extends SubsystemBase {
  private final CANSparkMax m_elevatorMotor = new CANSparkMax(14, MotorType.kBrushless);
  private final RelativeEncoder m_elevatorEncoder = m_elevatorMotor.getEncoder();
  private PIDController m_elevatorPID = new PIDController(LifterConstants.kP, 0, 0);
  private SlewRateLimiter m_elevatorRateLimiter = new SlewRateLimiter(2);
  private double kElevatorSetpoint = -0.05;

  private DigitalInput LimitSwitch = new DigitalInput(0);
  private boolean hasReset = false;

  /** Creates a new PIDElevatorSubsystem. */
  public PIDElevatorSubsystem() {
    m_elevatorMotor.setIdleMode(IdleMode.kBrake);
    m_elevatorEncoder.setPositionConversionFactor(LifterConstants.kFinalConversionFactor);
    m_elevatorEncoder.setVelocityConversionFactor(LifterConstants.kFinalConversionFactor / 60);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ElevatorPosition", m_elevatorEncoder.getPosition());
    SmartDashboard.putBoolean("limit", LimitSwitch.get());
    SmartDashboard.putNumber("ElevatorSetpoint", kElevatorSetpoint);

    SmartDashboard.putBoolean("Elevator Has Reset", hasReset);

    if (LimitSwitch.get() == false && hasReset == false) {
      m_elevatorEncoder.setPosition(0);
      hasReset = true;
    }

    if (hasReset == true) {
      Double output = m_elevatorRateLimiter.calculate(
          MathUtil.clamp((m_elevatorPID.calculate(m_elevatorEncoder.getPosition(), kElevatorSetpoint)), -0.5, 0.5));
      m_elevatorMotor.set(output);
    } else {
      m_elevatorMotor.set(-0.2);
    }
  }

  public void reset() {
    hasReset = false;
  }

  public boolean atHome() {
    return !LimitSwitch.get();
  }

  public void setPos(double setpoint) {
    this.kElevatorSetpoint = setpoint;
  }

  public double getPosition() {
    return m_elevatorEncoder.getPosition();
  }

  public boolean hasReset() {
    return hasReset;
  }

}
