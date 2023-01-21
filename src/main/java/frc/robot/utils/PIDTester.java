package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PIDTester extends SubsystemBase {
    double kp, ki, kd;
    public static int i = 10;

    PIDController PidController;
    String name;
    public PIDTester(PIDController pidController, String name, double kp, double ki, double kd) {
       this.kp = kp;
       this.ki = ki;
       this.kd = kd;
       this.PidController = pidController;
       this.name = name;
       SmartDashboard.putNumber(name + "PID_kp", this.kp);
       SmartDashboard.putNumber(name + "PID_ki", this.ki);
       SmartDashboard.putNumber(name + "PID_kd", this.kd);

    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        double p =SmartDashboard.getNumber(name + "PID_kp", this.kp);
        double i = SmartDashboard.getNumber(name + "PID_ki", this.ki);
        double d = SmartDashboard.getNumber(name + "PID_kd", this.kd);

        if ((p != this.kp)) {this.PidController.setP(p); this.kp = p;}
        if ((i != this.ki)) {this.PidController.setI(i); this.ki = i;}
        if ((d != this.kd)) {this.PidController.setD(d); this.kd = d;}

        SmartDashboard.putNumber(name + "PID SYSTEM_kp", p);
        SmartDashboard.putNumber(name + "PID SYSTEM_ki", i);
        SmartDashboard.putNumber(name + "PID SYSTEM_kd", d);
}

    // @Override
    // public void execute() {
    //     double p =SmartDashboard.getNumber(name + "PID_kp", this.kp);
    //     double i = SmartDashboard.getNumber(name + "PID_ki", this.ki);
    //     double d = SmartDashboard.getNumber(name + "PID_kd", this.kd);

    //     if ((p != this.kp)) {this.PidController.setP(p); this.kp = p;}
    //     if ((i != this.ki)) {this.PidController.setI(i); this.ki = i;}
    //     if ((d != this.kd)) {this.PidController.setD(d); this.kd = d;}
        
    // }

    // @Override
    // public boolean isFinished() {
    //     return false;
    // }
    
}
