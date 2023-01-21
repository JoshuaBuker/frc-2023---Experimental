// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.DoNothing;
import frc.robot.commands.Autonomous.Autonomous_Drive_And_Balance;
import frc.robot.commands.Autonomous.Autonomous_Travel_To_Then_Balance;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  // This is where I create all of the new subsystems. You can also do this in
  // RobotContainer, but it is up to you. However, Robot. is faster to type than
  // RobotContainer.
  // I created objects for both the Pneumatics hub and PDH because i want to be
  // able to clear sticky faults cause my dad did not like them flashing red. This
  // is unneccessary
  public static PneumaticHub hub = new PneumaticHub(3);
  public static PowerDistribution powerhub = new PowerDistribution(1, PowerDistribution.ModuleType.kRev);

  public static SendableChooser<Command> sendablechooser = new SendableChooser<Command>();

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings, and put our autonomous chooser on the dashboard.
    RobotContainer m_robotContainer = new RobotContainer();
  
    DrivetrainSubsystem m_drivetrainSubsystem = RobotContainer.getDrivetrain();
    m_drivetrainSubsystem.setPitchOffset();

    // Clears afformentioned sticky faults
    hub.clearStickyFaults();
    powerhub.clearStickyFaults();
    // Creates USB camera and starts streaming so the smartdashboard can get the
    // feed.
    UsbCamera cam = CameraServer.startAutomaticCapture();
    // FRC has a limit on resolution, so gotta use small ones rather than 1080p.
    cam.setResolution(160, 120);

    sendablechooser.setDefaultOption("Do nothing", new DoNothing());
    sendablechooser.addOption("Drive Till Balance", new Autonomous_Drive_And_Balance());
    sendablechooser.addOption("Travel Then Balance", new Autonomous_Travel_To_Then_Balance());
    SmartDashboard.putData("Autonomous", sendablechooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and
   * test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = sendablechooser.getSelected();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
}
