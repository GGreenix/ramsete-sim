// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPRamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot periodic
 * methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private static final double KS = 0;

private static final double KV = 0;

private static final double KA = 0;

// The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  XboxController m_driverController =
      new XboxController(Constants.OIConstants.kDriverControllerPort);
      JoystickButton _b1 = new JoystickButton(m_driverController,1);

    private PathPlannerTrajectory traj1 =  getTrajToCenter();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
      
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    -m_driverController.getRawAxis(1), m_driverController.getLeftTriggerAxis()),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
 
  public void configureButtonBindings() {
    _b1.whenPressed(new SelectCommand(this::followTrajectoryCommand));
  }
  
  public DriveSubsystem getRobotDrive() {
    return m_robotDrive;
  }
  
  
  public void zeroAllOutputs() {
    m_robotDrive.tankDriveVolts(0, 0);
  }
  public PathPlannerTrajectory getTrajToCenter(){
    // System.out.println(m_robotDrive.getPose());
    // System.out.println(m_robotDrive.getPose());
    
    return PathPlanner.generatePath(
        new PathConstraints(3,1), 
        new PathPoint(new Translation2d(m_robotDrive.getPose().getX(), m_robotDrive.getPose().getY()), m_robotDrive.getPose().getRotation()), // position, heading
        new PathPoint(new Translation2d(8.0,4.0), m_robotDrive.getPose().getRotation()) // position, heading
        
    );
  }
  public Command followTrajectoryCommand()
{
  System.out.println("recomfigured");

    return new SequentialCommandGroup(new InstantCommand(() -> {
        
        System.out.println("");
      })
       ,new PPRamseteCommand(
        getTrajToCenter(), 
            m_robotDrive::getPose, // Pose supplier
            new RamseteController(),
            new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts,
                Constants.DriveConstants.kvVoltSecondsPerMeter,
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
            Constants.DriveConstants.kDriveKinematics,// DifferentialDriveKinematics
            m_robotDrive::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
            new PIDController(3, 0, 0), // Left controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
            new PIDController(3, 0, 0), // Right controller (usually the same values as left controller)
            m_robotDrive::tankDriveVolts, // Voltage biconsumer
            m_robotDrive // Requires this drive subsystem
        ));
}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      // This will load the file "Example Path.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
PathPlannerTrajectory examplePath = PathPlanner.loadPath("New New New Path", new PathConstraints(4, 3));

// This trajectory can then be passed to a path follower such as a PPSwerveControllerCommand
// Or the path can be sampled at a given point in time for custom path following

// Sample the state of the path at 1.2 seconds
PathPlannerState exampleState = (PathPlannerState) examplePath.sample(1.2);

// Print the velocity at the sampled time
System.out.println(exampleState.velocityMetersPerSecond);
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts,
                Constants.DriveConstants.kvVoltSecondsPerMeter,
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
            Constants.DriveConstants.kDriveKinematics,
            1);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(Constants.DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at (1, 2) facing the +X direction
            new Pose2d(3, 4, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(1, 3), new Translation2d(0, 1)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(10, 2, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_robotDrive::getPose,
            new RamseteController(
                Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts,
                Constants.DriveConstants.kvVoltSecondsPerMeter,
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
            Constants.DriveConstants.kDriveKinematics,
            m_robotDrive::getWheelSpeeds,
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_robotDrive::tankDriveVolts,
            m_robotDrive);

    // Reset odometry to starting pose of trajectory.
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }
}
