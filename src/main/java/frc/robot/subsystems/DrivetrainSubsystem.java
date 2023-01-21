// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {

  public class Ratio {
    private double xValue;
    private double yValue;

    public double getxValue() {
      return xValue;
    }

    public void setxValue(double xValue) {
      this.xValue = xValue;
    }

    public double getyValue() {
      return yValue;
    }

    public void setyValue(double yValue) {
      this.yValue = yValue;
    }
  }
  // This is made purely from the SDS Template so I will try my best to explain
  // everything to the best of my ability

  private static double currentPitchOffset;

  // This varible represents the MAX_VOLTAGE the motors will use when operating.
  // Lower means slower, but may help with brownouts
  public static final double MAX_VOLTAGE = 12.0;

  // Calcuates Max Velocity using Motor RPM / 60. Drive Reduction is found from
  // the Gear Ratios from the Swerve Modules. Then gets cirfumfrence of wheels
  // using wheel diameter and pi
  public final static double MAX_VELOCITY_METERS_PER_SECOND = 5676.0 / 60.0 *
      SdsModuleConfigurations.MK4_L2.getDriveReduction() *
      SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

  // Calucates the max speed of the Angle Motors using the drive motor max speed
  // with more math
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
      Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  // Creates kinematics object using the robots dimensions and the WPI
  // SwerveDriveKinematics class. More research is needed to understand how it
  // works
  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      // Front left
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Front right
      new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back left
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
      // Back right
      new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

  // Creates Navx object using AHRS class and the MXP port on the RoboRio
  private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

  public double getPitchWithOffset() {
    return m_navx.getPitch() - currentPitchOffset;
  }

  public double getOffset() {
    return currentPitchOffset;
  }

  public void setPitchOffset() {
    currentPitchOffset = m_navx.getPitch();
  }

  public AHRS getNavx() {
    return m_navx;
  }

  // Declares names of the modules that will be used
  // These are our modules. We initialize them in the constructor.
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  // Creates a ChassisSpeed varible that will be updated with whatever is put into
  // the drive command
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem() {

    
    // Creates Shuffleboard tab for the drivetrain values created from the library's
    // Module builder
    final ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

    // ========================= Module Factory
    // =====================================

    m_frontLeftModule = Mk4SwerveModuleHelper.createNeo(
        // This parameter is optional, but will allow you to see the current state of
        // the module on the dashboard.
        tab.getLayout("Front Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(0, 0),
        // This can either be STANDARD or FAST depending on your gear configuration
        Mk4SwerveModuleHelper.GearRatio.L2,
        // This is the ID of the drive motor
        FRONT_LEFT_MODULE_DRIVE_MOTOR,
        // This is the ID of the steer motor
        FRONT_LEFT_MODULE_STEER_MOTOR,
        // This is the ID of the steer encoder
        FRONT_LEFT_MODULE_STEER_ENCODER,
        // This is how much the steer encoder is offset from true zero (In our case,
        // zero is facing straight forward)
        FRONT_LEFT_MODULE_STEER_OFFSET);

    // We will do the same for the other modules
    m_frontRightModule = Mk4SwerveModuleHelper.createNeo(
        tab.getLayout("Front Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(2, 0),
        Mk4SwerveModuleHelper.GearRatio.L2,
        FRONT_RIGHT_MODULE_DRIVE_MOTOR,
        FRONT_RIGHT_MODULE_STEER_MOTOR,
        FRONT_RIGHT_MODULE_STEER_ENCODER,
        FRONT_RIGHT_MODULE_STEER_OFFSET);

    m_backLeftModule = Mk4SwerveModuleHelper.createNeo(
        tab.getLayout("Back Left Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(4, 0),
        Mk4SwerveModuleHelper.GearRatio.L2,
        BACK_LEFT_MODULE_DRIVE_MOTOR,
        BACK_LEFT_MODULE_STEER_MOTOR,
        BACK_LEFT_MODULE_STEER_ENCODER,
        BACK_LEFT_MODULE_STEER_OFFSET);

    m_backRightModule = Mk4SwerveModuleHelper.createNeo(
        tab.getLayout("Back Right Module", BuiltInLayouts.kList)
            .withSize(2, 4)
            .withPosition(6, 0),
        Mk4SwerveModuleHelper.GearRatio.L2,
        BACK_RIGHT_MODULE_DRIVE_MOTOR,
        BACK_RIGHT_MODULE_STEER_MOTOR,
        BACK_RIGHT_MODULE_STEER_ENCODER,
        BACK_RIGHT_MODULE_STEER_OFFSET);
  }

  // ============================= Gyroscope =================================

  // Sets the current direction as forward for field oriented driving
  public void zeroGyroscope() {
    m_navx.zeroYaw();
  }

  // Gets current Gyroscope rotation. This value will be used to figure out how
  // far each wheel needs to spin to reach a certain direction.
  public Rotation2d getGyroscopeRotation() {

    if (m_navx.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return Rotation2d.fromDegrees(m_navx.getFusedHeading());
    }

    // We have to invert the angle of the NavX so that rotating the robot
    // counter-clockwise makes the angle increase.
    return Rotation2d.fromDegrees(360.0 - m_navx.getYaw());
  }

  // Basic drive command. To call it do - drive(new ChassisSpeeds(0.0, 0.0, 0.0))
  // and replace the 0.0s with whatever value you want
  public void drive(final ChassisSpeeds chassisSpeeds) {
    m_chassisSpeeds = chassisSpeeds;
  }

  // ====================== Periodic Function ===============================

  // Constantly sets the wheels to the required angle. Since this is in a periodic
  // function, it will happen 50 times a second. Or every 20ms.
  @Override
  public void periodic() {
    final SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[0].angle.getRadians());
    m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[1].angle.getRadians());
    m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[2].angle.getRadians());
    m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
        states[3].angle.getRadians());

    SmartDashboard.putNumber("Pitch", getPitchWithOffset());
  }

  // ============= Swerve Module Getter Methods =======================

  public SwerveModule getFrontLeftModule() {
    return m_frontLeftModule;
  }

  public SwerveModule getFrontRightModule() {
    return m_frontRightModule;
  }

  public SwerveModule getBackLeftModule() {
    return m_backLeftModule;
  }

  public SwerveModule getBackRightModule() {
    return m_backRightModule;
  }

  // =============== Distance Traveled Methods ======================

  public double calculateHypotenuse(double x, double y) {
    return Math.sqrt((x * x) + (y * y));
  }

  public double calculateRequiredTime(double distance, double velocity) {
    return Math.abs(distance / velocity);
  }

  public double getAverageVelocity() {
    return Math.abs((getFrontLeftModule().getDriveVelocity() + getFrontRightModule().getDriveVelocity()
        + getBackLeftModule().getDriveVelocity() + getBackRightModule().getDriveVelocity()) / 4);
  }

  public Ratio calculateRatio(double x, double y, double velocity) {

    Ratio ratio = new Ratio();

    if (x == 0 && y == .0) {
      ratio.setxValue(0.0);
      ratio.setyValue(0.0);
    } else if (x == 0.0) {

      if (y < 0) {
        ratio.setxValue(0.0);
        ratio.setyValue(velocity * -1);
      } else {
        ratio.setxValue(0.0);
        ratio.setyValue(velocity);
      }
    } else if (y == 0.0) {
      if (y < 0) {
        ratio.setxValue(velocity * -1);
        ratio.setyValue(0.0);
      } else {
        ratio.setxValue(velocity);
        ratio.setyValue(0.0);
      }
    } else {

      if (x > y) {
        ratio.setxValue(velocity);
        ratio.setyValue(velocity * (y / x));
      } else if (y > x) {
        ratio.setxValue(velocity * (y / x));
        ratio.setyValue(velocity);
      } else {
        ratio.setxValue(velocity);
        ratio.setyValue(velocity);
      }

    }
    return ratio;

  }

  public void driveDistance(double distanceX, double distanceY, double wantedVelocity) {
    Timer timer = new Timer();
    Ratio ratio = calculateRatio(distanceX, distanceY, wantedVelocity);
    double calculatedRequiredTime = calculateRequiredTime(calculateHypotenuse(distanceX, distanceY),
        wantedVelocity * MAX_VELOCITY_METERS_PER_SECOND);

    timer.start();
    while (timer.get() <= calculatedRequiredTime) {
      drive(new ChassisSpeeds(ratio.getxValue(), ratio.getyValue(), 0.0));
    }
    timer.stop();
  }
}