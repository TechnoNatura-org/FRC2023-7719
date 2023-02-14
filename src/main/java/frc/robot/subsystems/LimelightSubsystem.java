// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

// import org.opencv.photo
// import org.photonvision.PhotonUtils;

// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

  PhotonCamera camera = new PhotonCamera("gloworm");

  public LimelightSubsystem() {
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("yaw", MCgetYaw());
  }

  // public double getVisionDistance(){
  // var result = camera.getLatestResult();
  // if (result.hasTargets()){
  // return PhotonUtils.calculateDistanceToTargetMeters(
  // 0.93,
  // 2.6416,
  // Units.degreesToRadians(25),
  // Units.degreesToRadians(result.getBestTarget().getPitch()));
  // } else {
  // return 0;
  // }
  // }

  public boolean targetDetected() {
    var result = camera.getLatestResult();
    return result.hasTargets();
  }

  public double MCgetYaw() {
    var result = camera.getLatestResult();
    return result.getBestTarget().getYaw();
  }

  public double MCgetPitch() {
    var result = camera.getLatestResult();
    return result.getBestTarget().getPitch();
  }
}
