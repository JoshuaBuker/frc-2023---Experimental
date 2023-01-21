
package frc.robot.commands.Autonomous;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Driving.AutoBalance;
import frc.robot.subsystems.DrivetrainSubsystem;

public class Autonomous_Drive_And_Balance extends CommandBase {

  DrivetrainSubsystem m_DrivetrainSubsystem = RobotContainer.getDrivetrain();
  private double currentPitch;

  public Autonomous_Drive_And_Balance() {

    addRequirements(RobotContainer.getDrivetrain());

  }

  @Override
  public void initialize() {

    m_DrivetrainSubsystem.driveDistance(1, 1, .4);

    currentPitch = m_DrivetrainSubsystem.getPitchWithOffset();

    while (currentPitch < 1 && currentPitch > -1) {
      m_DrivetrainSubsystem.drive(new ChassisSpeeds(0.3, 0.0, 0.0));
    }

  }

  @Override
  public void execute() {

    new AutoBalance();

  }

  @Override
  public void end(boolean interrupted) {
    m_DrivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
