// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {

  private final CANSparkMax driveMotor;
  private final CANSparkMax turningMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turningEncoder;

  private final PIDController turningPidController;

  private final CANCoder absoluteEncoder;

  private final String moduleName; 

  /** Creates a new SwerveModule. */
  public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorInverted, int absoluteEncoderId,
      double absoluteEncoderOffset, String moduleName) {

    this.moduleName = moduleName;

    absoluteEncoder = new CANCoder(absoluteEncoderId);
    absoluteEncoder.configFactoryDefault();
    absoluteEncoder.configMagnetOffset(absoluteEncoderOffset);

    driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    driveMotor.restoreFactoryDefaults();
    turningMotor.restoreFactoryDefaults();

    driveMotor.setInverted(driveMotorInverted);
    turningMotor.setInverted(DriveConstants.kTurnMotorInverted);

    driveEncoder = driveMotor.getEncoder();
    turningEncoder = turningMotor.getEncoder();

    driveEncoder.setPositionConversionFactor(ModuleConstants.kDrivePositionConversionFactor);
    driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveVelocityConversionFactor);

    turningEncoder.setPositionConversionFactor(ModuleConstants.kTurnPositionConversionFactor);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurnVelocityConversionFactor);

    turningPidController = new PIDController(ModuleConstants.kPTurn, 0, 0);
    turningPidController.enableContinuousInput(-Math.PI, Math.PI);
  
    driveMotor.setIdleMode(IdleMode.kBrake);
    // System.out.print("getTurningPosition()");
    // System.out.println(getTurningPosition());
    // turningMotor.set(turningPidController.calculate(getTurningPosition(), 1));

    resetEncoders();
    
  }

  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  public SwerveModulePosition getModulePostion() {
    return new SwerveModulePosition(getDrivePosition(), new Rotation2d(getTurningPosition()));
  }

  public double getTurningPosition() {
    return turningEncoder.getPosition();
  }

  public double getDriveVelocity() {
    // driveEncoder. 
    return driveEncoder.getVelocity();
  }

  public double getTurningVelocity() {
    return turningEncoder.getVelocity();
  }

  public double getAbsoluteEncoderRad() {
    return -1 * Math.toRadians(absoluteEncoder.getAbsolutePosition());
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turningEncoder.setPosition(getAbsoluteEncoderRad());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
  }

  public void setDesiredState(SwerveModuleState state) {

    if (Math.abs(state.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }

    state = SwerveModuleState.optimize(state, getState().angle);
    driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

    double turningSpeed = turningPidController.calculate(getTurningPosition(), state.angle.getRadians());
    turningMotor.set(turningSpeed);
    SmartDashboard.putString("Swerve[ " + absoluteEncoder + " ]", state.toString());
    SmartDashboard.putNumber(moduleName + " Turning", turningSpeed);

  }

  public void stop() {
    driveMotor.set(0);
    turningMotor.set(0);
    // turningMotor.get()
  }
  
  public double getWheelRotationInDegrees() {
    return new Rotation2d(getTurningPosition()).getDegrees();
  }
}
