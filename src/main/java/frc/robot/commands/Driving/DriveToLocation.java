// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driving;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem.Ratio;

public class DriveToLocation extends CommandBase {
  DrivetrainSubsystem m_drivetrainSubsystem = RobotContainer.getDrivetrain();
  Ratio ratio;

  double distanceX = 0.5;
  double distanceY = 0.5;
  double wantedVelocity = 0.5;
  Timer timer = new Timer();
  double calculatedRequiredTime;

  /** Creates a new DriveToLocation. */
  public DriveToLocation() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ratio = m_drivetrainSubsystem.calculateRatio(distanceX, distanceY, wantedVelocity);
    calculatedRequiredTime = m_drivetrainSubsystem.calculateRequiredTime(m_drivetrainSubsystem.calculateHypotenuse(distanceX, distanceY), wantedVelocity * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND);
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    m_drivetrainSubsystem.drive(new ChassisSpeeds(ratio.getxValue(), ratio.getyValue(), 0.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0,0.0,0.0));
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() <= calculatedRequiredTime;
  }
}
