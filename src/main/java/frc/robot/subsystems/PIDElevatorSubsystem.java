package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PIDElevatorSubsystem extends SubsystemBase {
    private DigitalInput limitSwitch = new DigitalInput(0);

    private final CANSparkMax m_lift = new CANSparkMax(Constants.LifterConstants.kLiftChannel, MotorType.kBrushless);
    private RelativeEncoder m_liftEncoder = m_lift.getEncoder();

    private PIDController m_liftPID = new PIDController(Constants.LifterConstants.kP, Constants.LifterConstants.kI,
            Constants.LifterConstants.kD);

    private double pos = 0.0;
    private double speed = 0.0;
    private double cVal = 0.00;
    private double setPoint = 0.0;

    private boolean reachedTop = false;

    private double topPoint = Constants.LifterConstants.kSetPointTop;
    private double botPoint = Constants.LifterConstants.kSetPointBot;

    private boolean disableLift = true;

    private Supplier<Double> liftSupplier;

    public CommandBase toggleLift() {
        return runOnce(() -> {
            this.disableLift = !this.disableLift;
        });
    }

    public PIDElevatorSubsystem(Supplier<Double> liftSupplier) {
        this.liftSupplier = liftSupplier;

        m_lift.setIdleMode(IdleMode.kBrake);
        m_liftEncoder.setPositionConversionFactor(
                Constants.LifterConstants.kFinalGearRatio * Constants.LifterConstants.kSprocketCircumference);
        m_lift.setInverted(true);
        m_liftPID.setTolerance(0.005);

        m_liftEncoder.setPosition(0);
    }

    @Override
    public void periodic() {

        pos = m_liftEncoder.getPosition();
        SmartDashboard.putNumber("Lift pos", pos);

        // reached top resets
        if (limitSwitch.get() == false && reachedTop == false) {
            m_liftEncoder.setPosition(0);
            reachedTop = true;

            setPoint = 0.05;
            pos = m_liftEncoder.getPosition();
        }

        if (reachedTop == false) {
            m_lift.set(0.2);
        } else {
            runPID(pos, setPoint);
        }

        cVal = this.liftSupplier.get();

        if (cVal >= 0.01 /* && this.disableLift == false */ && Constants.status == "teleop") {

            if (pos > 0.83 && cVal < 0.01) {
                speed = 0.0;
            } else if (pos < 0.08 && cVal > -0.01) {
                speed = 0.0;
            } else {
                speed = Math.abs(cVal) > 0.1 ? -cVal : 0.0;
            }
            m_lift.set(speed);

            setPoint = m_liftEncoder.getPosition();

        } else {
            runPID(pos, setPoint);
        }

        // DriverStation.getEventName()
    }

    public void setPos(double targetSetpoint) {
        this.setPoint = targetSetpoint;
    }

    public void resetReachTop() {
        this.reachedTop = false;
    }

    public void runPID(double output, double setPoint) {
        var value = 0.0;
        value = m_liftPID.calculate(output, setPoint);
        value = MathUtil.clamp(value, -1.0, 1.0);
        m_lift.set(value);
    }

    public CommandBase setToTop() {
        return runOnce(() -> {
            var value = 0.0;
            value = m_liftPID.calculate(this.pos, topPoint);
            value = MathUtil.clamp(value, -1.0, 1.0);
            m_lift.set(value);
        });
    }

    public CommandBase setToBot() {
        return runOnce(() -> {
            var value = 0.0;
            value = m_liftPID.calculate(pos, botPoint);
            value = MathUtil.clamp(value, -1.0, 1.0);
            m_lift.set(value);
        });
    }

}
