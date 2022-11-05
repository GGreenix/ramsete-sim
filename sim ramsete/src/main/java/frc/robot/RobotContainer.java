// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot periodic
 * methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  

// The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  XboxController m_driverController =
      new XboxController(Constants.OIConstants.kDriverControllerPort);
      JoystickButton _b1 = new JoystickButton(m_driverController,1);

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
    _b1.whileActiveOnce(new SelectCommand(this::followTrajectoryCommand));
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
  System.out.println("scheduled");

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
 
}
