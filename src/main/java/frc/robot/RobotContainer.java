// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
// import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoTrajectoryFileNames;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.TeleOp;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.PIDTester;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // private final DriverStation driverStation = new DriverStation();
  private final SwerveSubsystem driveSubsystem = new SwerveSubsystem();
  private final XboxController joystick_1 = new XboxController(OIConstants.kDriverControllerPort);

  private PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  private PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  private PIDController thetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);
		
  private final PIDTester xControllTester = new PIDTester(xController, "xController", AutoConstants.kPXController, 0, 0);
  private final PIDTester yControllTester = new PIDTester(yController, "yController", AutoConstants.kPYController, 0, 0);
  private final PIDTester thetaControllTester = new PIDTester(thetaController, "thetaController", AutoConstants.kPYController, 0, 0);

  private final SendableChooser<JKAutoProfile> m_autoCommandChooser = new SendableChooser<>();


  public RobotContainer() {

    driveSubsystem.setDefaultCommand(new TeleOp(driveSubsystem, () -> -joystick_1.getLeftY(), () -> -joystick_1.getLeftX(), () -> joystick_1.getRightX(), () -> joystick_1.getRightBumper() ));
   
   
    JKAutoProfile empyyProfile = new JKAutoProfile();
    m_autoCommandChooser.setDefaultOption("Do nothing", empyyProfile);
    m_autoCommandChooser.addOption(AutoTrajectoryFileNames.POS_TOP_LEAVE, Autos.posTopLeave());

    SmartDashboard.putData("auto choices", m_autoCommandChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(joystick_1, XboxController.Button.kA.value).onTrue(Commands.runOnce(() -> {
      driveSubsystem.zeroHeading();
    }, driveSubsystem));

    // new JoystickButton(joystick_1, XboxController.Button.kB.value).onTrue(Commands.runOnce(() -> {
    //   driveSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));
    // }, driveSubsystem));

    // new JoystickButton(joystick_1, XboxController.Button.kY.value).onTrue(Commands.runOnce(() -> {
    //   driveSubsystem.ResetPos(xController, yController);
    // }, driveSubsystem));


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // DriverStation.getMatchTime()


    PathPlannerTrajectory PIDField = PathPlanner.loadPath("PID Straight 1 m", new PathConstraints(1.2, 1));

    
    
    // PIDField.a
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    PPSwerveControllerCommand pIDSwerveControllerCommand = new PPSwerveControllerCommand(PIDField, driveSubsystem::getPose2d, DriveConstants.kDriveKinematics, xController, yController, thetaController, driveSubsystem::setModuleState,false, driveSubsystem);

    return new SequentialCommandGroup(
      new InstantCommand( () -> {
        driveSubsystem.resetOdometry(PIDField.getInitialHolonomicPose());
      }),
      pIDSwerveControllerCommand,
      new InstantCommand(() -> {
        driveSubsystem.stopModules();
      })
    );
  }
}
