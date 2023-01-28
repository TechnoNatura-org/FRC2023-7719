package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveSubsystem;

public class WheelsRotationTester extends SubsystemBase {
    double fr = 0;
    double fl = 0;
    double rr = 0;
    double rl = 0;

    private SwerveSubsystem swerveSubsystem;
    public WheelsRotationTester(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        double[] wheelsRotation =swerveSubsystem.getWheelsRotationInDegrees();
        fl = wheelsRotation[0];
        fr = wheelsRotation[1];
        rl = wheelsRotation[2];
        rr = wheelsRotation[3];
        SmartDashboard.putNumber("fr rotation", fr);
        SmartDashboard.putNumber("fl rotation" , fl);
        SmartDashboard.putNumber("rr", rr);
        SmartDashboard.putNumber("rl rotation", rl);

    }
    
    @Override
    public void periodic() {
        double flS = SmartDashboard.getNumber("fl rotation" , fl);
        double frS = SmartDashboard.getNumber("fr rotation", fr);
        double rlS = SmartDashboard.getNumber("rl rotation", rl);
        double rrS = SmartDashboard.getNumber("rr", rr);

        double[] wheelsRotation = this.swerveSubsystem.getWheelsRotationInDegrees();
        
        if (flS != wheelsRotation[0] || frS != wheelsRotation[1] || rlS != wheelsRotation[2] || rrS != wheelsRotation[3]) {
            this.swerveSubsystem.setOutput(0, new double[] {
                flS,
                frS,
                rlS,
                rrS,
            });
        }

        SmartDashboard.putNumber("fr rotation", fr);
        SmartDashboard.putNumber("fl rotation" , fl);
        SmartDashboard.putNumber("rr", rr);
        SmartDashboard.putNumber("rl rotation", rl);

        
    }
}
