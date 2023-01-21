// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Driving;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionCamera;
import oi.limelightvision.limelight.frc.LimeLight;

public class Align_With_Apriltag extends CommandBase {

  VisionCamera m_visionCamera = RobotContainer.getVisioncamera();
  LimeLight limelight = RobotContainer.getVisioncamera().getLimeLight();
  DrivetrainSubsystem m_drivetrainSubsystem = RobotContainer.getDrivetrain();

  //double kp_X = -0.5;
  double kp_X = 0.0;
  double ki_X = 0.0;
  double kd_X = 0.0;

  double kp_Y = 0.05;
  double ki_Y = 0.01;
  double kd_Y = 0.0;

  double kp_Z = -0.01;
  double ki_Z = 0.0;
  double kd_Z = 0.0;

  PIDController pidX = new PIDController(kp_X, ki_X, kd_X);
  PIDController pidY = new PIDController(kp_Y, ki_Y, kd_Y);
  PIDController pidZ = new PIDController(kp_Z, ki_Z, kd_Z);

  public Align_With_Apriltag() {
    addRequirements(m_drivetrainSubsystem, m_visionCamera);
  }

  @Override
  public void initialize() {
    m_visionCamera.enableApriltagTargeting();
    pidX.setTolerance(0.25);
    pidY.setTolerance(0.25);
    pidZ.setTolerance(0.25);
  }

  @Override
  public void execute() {

    if (limelight.getIsTargetFound()) {
      if (limelight.getTagID() == Constants.AprilTag.BlueSubstaion.id) {

        m_drivetrainSubsystem.drive(new ChassisSpeeds(
            pidX.calculate(limelight.getdegVerticalToTarget(), 0),
            pidY.calculate(limelight.getdegRotationToTarget(), 0),
            pidZ.calculate(limelight.getTagYaw(), 0)));

      } else if (limelight.getTagID() == Constants.AprilTag.RedSubstation.id) {

        m_drivetrainSubsystem.drive(new ChassisSpeeds(
            pidX.calculate(limelight.getdegVerticalToTarget(), 15),
            pidY.calculate(limelight.getdegRotationToTarget(), 0),
            pidZ.calculate(limelight.getTagYaw(), 0)));

      }
    }

  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    pidX.close();
    pidY.close();
    pidZ.close();
  }

  @Override
  public boolean isFinished() {
    return pidX.atSetpoint() && pidY.atSetpoint() && pidZ.atSetpoint();
  }
}
