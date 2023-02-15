// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoTrajectoryFileNames;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleOpCmd;
import frc.robot.commands.teleopCmd.ArmPositionJoystick;
import frc.robot.commands.teleopCmd.ArmPositioningCmd;
import frc.robot.commands.teleopCmd.ElevatorJoystickCmd;
import frc.robot.commands.teleopCmd.ExtenderJoystickCmd;
import frc.robot.commands.teleopCmd.intakeSetCmd;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PIDArmSubsystem;
import frc.robot.subsystems.PIDElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.List;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

        private final XboxController xboxController = new XboxController(OIConstants.kDriverControllerPort);
        private final XboxController CoOpDriver = new XboxController(1);

        // private final DriverStation driverStation = new DriverStation();
        private final SwerveSubsystem driveSubsystem = new SwerveSubsystem();
        private final ExtenderSubsystem extenderSubsystem = new ExtenderSubsystem();
        // private final PIDElevatorSubsystem2 elevatorSubsystem = new
        // PIDElevatorSubsystem2(() -> CoOpDriver.getLeftY());
        public final PIDElevatorSubsystem elevatorSubsystem = new PIDElevatorSubsystem();
        public final PIDArmSubsystem armSubsystem = new PIDArmSubsystem();
        private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

        private final SendableChooser<JKAutoProfile> m_autoCommandChooser = new SendableChooser<>();
        private final SendableChooser<CommandBase> m_teleopCommandChooser = new SendableChooser<>();

        private final ArmPositioningCmd armJoystickCmd = new ArmPositioningCmd(armSubsystem,
                        () -> -CoOpDriver.getLeftX());

        public RobotContainer() {

                // elevatorSubsystem.resetReachTop();

                armSubsystem.setDefaultCommand(armJoystickCmd);

                new JoystickButton(
                                CoOpDriver, XboxController.Button.kA.value).onTrue(
                                                new ArmPositionJoystick(armSubsystem,
                                                                ManipulatorConstants.kFrontGroundPosition,
                                                                () -> -CoOpDriver.getLeftX()));
                new JoystickButton(
                                CoOpDriver, XboxController.Button.kX.value).onTrue(
                                                new ArmPositionJoystick(armSubsystem,
                                                                ManipulatorConstants.kTopNodePosition,
                                                                () -> -CoOpDriver.getLeftX()));
                new JoystickButton(
                                CoOpDriver, XboxController.Button.kB.value)
                                .onTrue(new ArmPositionJoystick(armSubsystem,
                                                ManipulatorConstants.kRearGroundPosition,
                                                () -> -CoOpDriver.getLeftX()));
                new JoystickButton(
                                CoOpDriver, XboxController.Button.kA.value)
                                .onTrue(new ArmPositionJoystick(armSubsystem,
                                                ManipulatorConstants.kGroundZeroPosition,
                                                () -> -CoOpDriver.getLeftX()));

                elevatorSubsystem.setDefaultCommand(new ElevatorJoystickCmd(elevatorSubsystem, () -> CoOpDriver
                                .getRightY()));
                extenderSubsystem.setDefaultCommand(
                                new ExtenderJoystickCmd(extenderSubsystem, () -> CoOpDriver.getPOV()));

                driveSubsystem.setDefaultCommand(new TeleOpCmd(driveSubsystem, () -> -xboxController.getLeftY(),
                                () -> -xboxController.getLeftX(), () -> -xboxController.getRightX(),
                                () -> xboxController.getRightBumper(),
                                () -> CoOpDriver.getRightTriggerAxis()));

                JKAutoProfile empyyProfile = new JKAutoProfile(armSubsystem, extenderSubsystem, elevatorSubsystem,
                                new InstantCommand(() -> {

                                }));

                m_autoCommandChooser.addOption("Engage ", Autos.engageTesting(
                                armSubsystem, extenderSubsystem, elevatorSubsystem, driveSubsystem));
                m_autoCommandChooser.addOption("Engage Moving", Autos.engageTestingMoving(
                                armSubsystem, extenderSubsystem, elevatorSubsystem, driveSubsystem));

                m_autoCommandChooser.addOption("Arm testing", Autos.armTesting(
                                armSubsystem, extenderSubsystem, elevatorSubsystem));

                m_autoCommandChooser.setDefaultOption("Do nothing", empyyProfile);
                m_autoCommandChooser.addOption(AutoTrajectoryFileNames.POS_TOP_LEAVE,
                                Autos.posTopLeave(AutoTrajectoryFileNames.POS_TOP_LEAVE, driveSubsystem, armSubsystem,
                                                extenderSubsystem, elevatorSubsystem));
                m_autoCommandChooser.addOption(AutoTrajectoryFileNames.POS_TOP_DOCK,
                                Autos.posTopDock(AutoTrajectoryFileNames.POS_TOP_DOCK, driveSubsystem, armSubsystem,
                                                extenderSubsystem, elevatorSubsystem));

                m_autoCommandChooser.addOption("PID Straight 1 m", Autos.PIDSTester("PID Straight 2 m", driveSubsystem,
                                armSubsystem, extenderSubsystem, elevatorSubsystem));
                m_autoCommandChooser.addOption("PID Straight Backward",
                                Autos.PIDSTester("PID Straight 3 m 180", driveSubsystem,
                                                armSubsystem, extenderSubsystem, elevatorSubsystem));
                m_autoCommandChooser.addOption("Mid Engage Testing",
                                Autos.PIDSTester("Mid Dock Testing", driveSubsystem,
                                                armSubsystem, extenderSubsystem, elevatorSubsystem));

                SmartDashboard.putData("auto choices", m_autoCommandChooser);

                // Configure the button bindings
                configureButtonBindings();
        }

        public void teleOpInit() {
                armJoystickCmd.teleOpInit();
        }

        /**
         * Use this method to define your button->command mappings. Buttons can be
         * created by
         * instantiating a {@link GenericHID} or one of its subclasses ({@link
         * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
         * passing it to a
         * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
         */
        private void configureButtonBindings() {

                // new JoystickButton(CoOpDriver,
                // Button.kOptions.value).onTrue(elevatorSubsystem.toggleLift());
                // new JoystickButton(CoOpDriver,
                // Button.kCircle.value).onTrue(elevatorSubsystem.setToTop())
                // .onFalse(elevatorSubsystem.setToBot());
                // new JoystickButton(xboxController,
                // XboxController.Button.kA.value).onTrue(Commands.runOnce(() -> {
                // driveSubsystem.zeroHeading();
                // }, driveSubsystem));

                new JoystickButton(
                                CoOpDriver, XboxController.Button.kLeftBumper.value)
                                .whileTrue(new intakeSetCmd(intakeSubsystem, 1.0));
                new JoystickButton(
                                CoOpDriver, XboxController.Button.kRightBumper.value)
                                .whileTrue(new intakeSetCmd(intakeSubsystem, -1.0));
                // new JoystickButton(
                // ps4Controller, XboxController.Button.kLeftBumper.value)
                // .whileTrue(Commands.runOnce(() -> armSubsystem.resetEncoder(),
                // armSubsystem));

                // new JoystickButton(joystick_1,
                // XboxController.Button.kB.value).onTrue(Commands.runOnce(() -> {
                // driveSubsystem.resetOdometry(new Pose2d(0,0, new Rotation2d(0)));
                // }, driveSubsystem));
        }

        public JKAutoProfile getAutonomousProfile() {
                AutoConstants.thetaController.enableContinuousInput(-Math.PI, Math.PI);
                return m_autoCommandChooser.getSelected();
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {

                AutoConstants.thetaController.enableContinuousInput(-Math.PI, Math.PI);

                ProfiledPIDController thtaControllerProfiled = new ProfiledPIDController(
                                AutoConstants.kPThetaController, 0,
                                AutoConstants.kDThetaController, new Constraints(0.5, 0.2));
                thtaControllerProfiled.enableContinuousInput(-Math.PI, Math.PI);

                Trajectory rotatingTrajectoryPID = TrajectoryGenerator.generateTrajectory(
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(3, 0, new Rotation2d(0)), DriveConstants.trajectoryConfig);
                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(rotatingTrajectoryPID,
                                driveSubsystem::getPose2d, DriveConstants.kDriveKinematics, AutoConstants.xController,
                                AutoConstants.yController, thtaControllerProfiled, driveSubsystem::setModuleState,
                                driveSubsystem);
                return new SequentialCommandGroup(
                                // new InstantCommand( () -> {
                                // driveSubsystem.resetOdometry(PIDField.getInitialHolonomicPose());
                                // }),
                                swerveControllerCommand
                // nes
                );
        }
}
