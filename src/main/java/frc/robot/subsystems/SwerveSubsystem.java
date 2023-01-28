package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
// import edu.wpi.first.wpilibj.ADXRS450_Gyro;
// import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule frontLeftModule = new SwerveModule(DriveConstants.kFrontLeftDriveMotorId,
            DriveConstants.kFrontLeftTurnMotorId, DriveConstants.kLeftDriveMotorInverted,
            DriveConstants.kFrontLeftCANCoderId, DriveConstants.kFrontLeftAbsEncoderOffset, "fl");
    private final SwerveModule rearLeftModule = new SwerveModule(DriveConstants.kRearLeftDriveMotorId,
            DriveConstants.kRearLeftTurnMotorId, DriveConstants.kLeftDriveMotorInverted,
            DriveConstants.kRearLeftCANCoderId, DriveConstants.kRearLeftAbsEncoderOffset, "rl");
    private final SwerveModule frontRightModule = new SwerveModule(DriveConstants.kFrontRightDriveMotorId,
            DriveConstants.kFrontRightTurnMotorId, DriveConstants.kRightDriveMotorInverted,
            DriveConstants.kFrontRightCANCoderId, DriveConstants.kFrontRightAbsEncoderOffset, "fr");
    private final SwerveModule rearRightModule = new SwerveModule(DriveConstants.kRearRightDriveMotorId,
            DriveConstants.kRearRightTurnMotorId, DriveConstants.kRightDriveMotorInverted,
            DriveConstants.kRearRightCANCoderId, DriveConstants.kRearRightAbsEncoderOffset, "rr");

    private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), gModulePositions());

    private final Field2d field2d = new Field2d();

    public SwerveSubsystem() {
        
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {

            }
        }).start();

        SmartDashboard.putData("Field", field2d);
        SmartDashboard.putData("Gyro", m_gyro);

        // this.zeroHeading();

        // System.out.println("SET TO 0 degrees");
        // this.setModuleState(new SwerveModuleState[]{
        // new SwerveModuleState(0, new Rotation2d(0)),
        // new SwerveModuleState(0, new Rotation2d(0)),
        // new SwerveModuleState(0, new Rotation2d(0)),
        // new SwerveModuleState(0, new Rotation2d(0))
        // });
        // System.out.println("DONE SET TO 0 degrees");

    }

    public void setZeroDegree() {
        this.zeroHeading();

        // System.out.println("SET TO 0 degrees");
        // this.setModuleState(new SwerveModuleState[]{
        // new SwerveModuleState(0, new Rotation2d(0)),
        // new SwerveModuleState(0, new Rotation2d(0)),
        // new SwerveModuleState(0, new Rotation2d(0)),
        // new SwerveModuleState(0, new Rotation2d(0))
        // });
        // System.out.println("DONE SET TO 0 degrees");
    }

    public SwerveModulePosition[] gModulePositions() {
        return new SwerveModulePosition[] {
                frontLeftModule.getModulePostion(),
                frontRightModule.getModulePostion(),
                rearLeftModule.getModulePostion(),
                rearRightModule.getModulePostion()
        };
    }

    public void zeroHeading() {
        m_gyro.reset();
    }

    public double getPitch() {
        return m_gyro.getXComplementaryAngle();
    }

    public double getHeading() {
        // IMUAxis p =  m_gyro.getYawAxis();
        // m_gyro
        // m_gyro
        return Math.IEEEremainder(m_gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose2d() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), gModulePositions(), pose);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot Heading", getHeading());
        odometry.update(getRotation2d(), gModulePositions());
        field2d.setRobotPose(getPose2d());
        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putString("robot location", getPose2d().getTranslation().toString());
        // SmartDashboard.putNumber("velocity", DriveConstants.kDriveKinematics.);
    }

    public void ResetPos(PIDController xController, PIDController yController) {
        ProfiledPIDController profiledThetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        profiledThetaController.enableContinuousInput(-Math.PI, Math.PI);

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(1, 1, new Rotation2d(90)), new Pose2d(2, 1, new Rotation2d(0))),
                DriveConstants.trajectoryConfig);

		SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            this::getPose2d,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            profiledThetaController,
            this::setModuleState,
            this);
            swerveControllerCommand.execute();
    
    }

    public void stopModules() {
        frontLeftModule.stop();
        rearLeftModule.stop();
        frontRightModule.stop();
        rearRightModule.stop();
    }


    public void setOutput(double output) {
        SwerveModuleState frontLeftModule = new SwerveModuleState(output, Rotation2d.fromDegrees(180));
        SwerveModuleState frontRightModule = new SwerveModuleState(output, Rotation2d.fromDegrees(180));
        SwerveModuleState rearLeftModule = new SwerveModuleState(output, Rotation2d.fromDegrees(180));
        SwerveModuleState rearRightModule = new SwerveModuleState(output, Rotation2d.fromDegrees(180));


        setModuleState(new SwerveModuleState[] {
            frontLeftModule,
            frontRightModule,
            rearLeftModule,
            rearRightModule
        });
    }

    public void setOutput(double output, double rotation[]) {
        SwerveModuleState frontLeftModule = new SwerveModuleState(output, Rotation2d.fromDegrees(rotation[0]));
        SwerveModuleState frontRightModule = new SwerveModuleState(output, Rotation2d.fromDegrees(rotation[1]));
        SwerveModuleState rearLeftModule = new SwerveModuleState(output, Rotation2d.fromDegrees(rotation[2]));
        SwerveModuleState rearRightModule = new SwerveModuleState(output, Rotation2d.fromDegrees(rotation[3]));

        setModuleState(new SwerveModuleState[] {
            frontLeftModule,
            frontRightModule,
            rearLeftModule,
            rearRightModule
        });
    }
    
    public void setModuleState(SwerveModuleState[] desiredStates) {
        // we use this to normalise all of desired states (module states)
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeftModule.setDesiredState(desiredStates[0]);
        frontRightModule.setDesiredState(desiredStates[1]);
        rearLeftModule.setDesiredState(desiredStates[2]);
        rearRightModule.setDesiredState(desiredStates[3]);

        SmartDashboard.putNumber("VEL FL", desiredStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("VEL FR", desiredStates[1].speedMetersPerSecond);
        SmartDashboard.putNumber("VEL LL", desiredStates[2].speedMetersPerSecond);
        SmartDashboard.putNumber("VEL LR", desiredStates[3].speedMetersPerSecond);

        SmartDashboard.putString("ROTATION FL", desiredStates[0].angle.toString());
        SmartDashboard.putString("ROTATION FR", desiredStates[1].angle.toString());
        SmartDashboard.putString("ROTATION LL", desiredStates[2].angle.toString());
        SmartDashboard.putString("ROTATION LR", desiredStates[3].angle.toString());

        return;
    }

    public CommandBase setWheelRotationCmd(double degrees[]) {
        return runOnce(() -> {
            setOutput(0, degrees);
        });
    }
    public void setWheelRotation(double degrees[]) {
        setOutput(0, degrees);
    }
    public double[] getWheelsRotationInDegrees() {
        return new double[] {
            frontLeftModule.getWheelRotationInDegrees(),
            frontRightModule.getWheelRotationInDegrees(),
            rearLeftModule.getWheelRotationInDegrees(),
            rearRightModule.getWheelRotationInDegrees(),
        };
    }
    
}
