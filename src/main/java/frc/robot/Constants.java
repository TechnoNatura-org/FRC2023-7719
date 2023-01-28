package frc.robot;

import com.pathplanner.lib.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.util.Units;
import frc.robot.utils.PIDTester;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {
	public static final class ModuleConstants {
		public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
		public static final double kDriveMotorGearRatio = 1 / 7.0;
		public static final double kTurnMotorGearRatio = 1 / 27.5;
		public static final double kDrivePositionConversionFactor = kDriveMotorGearRatio * kWheelDiameterMeters * Math.PI;
		public static final double kTurnPositionConversionFactor = kTurnMotorGearRatio * 2 * Math.PI;
		public static final double kDriveVelocityConversionFactor = kDrivePositionConversionFactor / 60;
		public static final double kTurnVelocityConversionFactor = kTurnPositionConversionFactor / 60;
		public static final double kPTurn = 0.5;
	}

	public static final class DriveConstants {


		public static final double kTrackWidth = 0.59;
		public static final double kWheelBase = 0.59;
		public static final Translation2d kFrontLeftPosition = new Translation2d(kWheelBase / 2, kTrackWidth / 2);
		public static final Translation2d kFrontRightPosition = new Translation2d(kWheelBase / 2, -kTrackWidth / 2);
		public static final Translation2d kRearLeftPosition = new Translation2d(-kWheelBase / 2, kTrackWidth / 2);
		public static final Translation2d kRearRightPosition = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);
		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				kFrontLeftPosition,
				kFrontRightPosition,
				kRearLeftPosition,
				kRearRightPosition
		);

        public static final TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(DriveConstants.kDriveKinematics);

		public static final int kFrontLeftDriveMotorId = 1;
		public static final int kFrontRightDriveMotorId = 4;
		public static final int kRearRightDriveMotorId = 5;
		public static final int kRearLeftDriveMotorId = 7;

		public static final int kFrontLeftTurnMotorId = 2;
		public static final int kFrontRightTurnMotorId = 3;
		public static final int kRearRightTurnMotorId = 6;
		public static final int kRearLeftTurnMotorId = 8;

		public static final boolean kTurnMotorInverted = true;

		public static final boolean kLeftDriveMotorInverted = true;
		public static final boolean kRightDriveMotorInverted = false;

		public static final int kFrontLeftCANCoderId = 9;
		public static final int kFrontRightCANCoderId = 10;
		public static final int kRearRightCANCoderId = 11;
		public static final int kRearLeftCANCoderId = 12;

		public static final double kFrontLeftAbsEncoderOffset = -320;
		public static final double kFrontRightAbsEncoderOffset = -81;
		public static final double kRearLeftAbsEncoderOffset = -205;
		public static final double kRearRightAbsEncoderOffset = -179;

		public static final double kPhysicalMaxSpeedMetersPerSecond = 4.8;
		public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 3 * 2 * Math.PI;
		public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
		public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = 
				kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
		public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
		public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
	}

	// public static 

	public static final class AutoConstants {
		public static final PathConstraints maxVelocityAcceleration = new PathConstraints(1.2,1.2);

        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        // public static final double kPXController = 1.5;
        // public static final double kPYController = 1.5;
		public static final double kPXController = 0.00001;// 0.00001
        public static final double kPYController = 0.00001;// 0.00001
        public static final double kPThetaController = 0.00001;// 3


  		public static PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  		public static PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  		public static PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
		
  		public static final PIDTester xControllTester = new PIDTester(xController, "xController", AutoConstants.kPXController, 0, 0);
  		public static final PIDTester yControllTester = new PIDTester(yController, "yController", AutoConstants.kPYController, 0, 0);
  		public static final PIDTester thetaControllTester = new PIDTester(thetaController, "thetaController", AutoConstants.kPYController, 0, 0);


		// public static final double kDXController = 0;
		// public static final double kDYController = 0;
		public static final double kDXController = 0.000010; // PID for making the speed perfect at 1.2 meters per second (max)
		public static final double kDYController = 0.000009; // PID for making the speed perfect at 1.2 meters per second (max)
		public static final double kDThetaController = 0;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

	public static final class OIConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kCoDriverControllerPort = 1;
		public static final double kDeadband = 0.05;
	}

	public static class AutoTrajectoryFileNames {
		public static final String POS_TOP_LEAVE = "posTopLeave";
		public static final String POS_TOP_DOCK = "posTopDock";
		public static final String POS_TOP_PICK = "posTopTakePiece";

		public static final String POS_MID_LEAVE = "posMidLeave";
		public static final String POS_MID_DOCK = "posMidDock";
		public static final String POS_MID_PICK = "posMidTakePiece";

		public static final String POS_LOW_LEAVE = "posLowLeave";
		public static final String POS_LOW_DOCK = "posLowDock";
		public static final String POS_LOW_PICK = "posLowTakePiece";
	}
}