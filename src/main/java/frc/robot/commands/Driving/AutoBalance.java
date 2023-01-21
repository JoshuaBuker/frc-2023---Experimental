// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driving;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoBalance extends CommandBase {

  DrivetrainSubsystem m_drivetrainSubsystem = RobotContainer.getDrivetrain();
  AHRS navx = m_drivetrainSubsystem.getNavx();

  private final float kP = 0.05f;
  private final float kI = 0.01f;
  private final float kD = 0.01f;

  PIDController pid = new PIDController(kP, kI, kD);

  /** Creates a new AutoBalance. */
  public AutoBalance() {

    addRequirements(RobotContainer.getDrivetrain());
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(
      pid.calculate(m_drivetrainSubsystem.getPitchWithOffset(), 0),
        RobotContainer.modifyAxis(RobotContainer.rightJoy.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND* -1,
        RobotContainer.modifyAxis(RobotContainer.rightJoy.getZ()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * -1));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    pid.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
