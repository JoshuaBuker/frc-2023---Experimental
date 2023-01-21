package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Driving.Align_With_Apriltag;
import frc.robot.commands.Driving.Align_With_Reflective;
import frc.robot.commands.Driving.AutoBalance;
import frc.robot.commands.Driving.DefaultDriveCommand;
import frc.robot.commands.Driving.DriveToLocation;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.VisionCamera;

public class RobotContainer {
  // This template creates the Drivetrain Subsystem here rather than in Robot.java
  private static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  //private static final Arm m_arm = new Arm(Constants.ARM_JOINT_ONE, Constants.ARM_JOINT_ONE_ENCODER);
  private static final VisionCamera m_visionCamera = new VisionCamera();
  

  // Joystick - Creates joystick objects using the ID number from constants and
  // the Joystick class
  public final Joystick leftJoy = new Joystick(Constants.leftJoystick);
  public final static Joystick rightJoy = new Joystick(Constants.rightJoystick);

  // Joystick button - Declares the names for each of the joystick buttons
  public JoystickButton rTrigger;
  public JoystickButton lTrigger;
  public JoystickButton lInside;
  public JoystickButton rInside;
  public JoystickButton lOutside;
  public JoystickButton rOutside;
  public JoystickButton rBottom;
  public JoystickButton lBottom;

  // GamePad - Declares the names for each of the gamepad buttons
  public final Joystick gamepad = new Joystick(Constants.gamepad);
  public JoystickButton gamepadX;
  public JoystickButton gamepadA;
  public JoystickButton gamepadY;
  public JoystickButton gamepadB;
  public JoystickButton gamepadStart;
  public JoystickButton gamepadSelect;
  public JoystickButton gamepadL1;
  public JoystickButton gamepadR1;
  public JoystickButton gamepadR3;
  public JoystickButton gamepadL3;

  // This is the RobotContainer method
  public RobotContainer() {
    // Sets default command for the drivetrainSubsystem. A default command will run
    // when nothing else is so it is perfect for a teleop driving command
    // The () -> - is a lambda function and allows the values of the joysticks to be
    // saved as the Double Suppliers in the defaultcommand
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_drivetrainSubsystem,
        () -> -modifyAxis(rightJoy.getY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(rightJoy.getX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(rightJoy.getZ()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / 2));

    // Runs the configureButtonBindings function created below.
    configureButtonBindings();
  }

  // Defines the function configureButtonBindings
  private void configureButtonBindings() {

    // Creates button objects for each button in a similar manner as the joysticks
    // from above
    // Joystick
    rTrigger = new JoystickButton(rightJoy, Constants.JoystickTriggerR);
    lTrigger = new JoystickButton(leftJoy, Constants.JoystickTriggerL);
    rInside = new JoystickButton(rightJoy, Constants.JoystickRightInside);
    lInside = new JoystickButton(leftJoy, Constants.JoystickLeftInside);
    rOutside = new JoystickButton(rightJoy, Constants.JoystickRightOutside);
    lOutside = new JoystickButton(leftJoy, Constants.JoystickLeftOutside);
    rBottom = new JoystickButton(rightJoy, Constants.JoystickRightBottom);
    lBottom = new JoystickButton(leftJoy, Constants.JoystickLeftBottom);
    // Same thing as joystick but for the gamepad buttons
    // Gamepad
    gamepadX = new JoystickButton(gamepad, Constants.GamepadX);
    gamepadA = new JoystickButton(gamepad, Constants.GamepadA);
    gamepadY = new JoystickButton(gamepad, Constants.GamepadY);
    gamepadB = new JoystickButton(gamepad, Constants.GamepadB);
    gamepadR1 = new JoystickButton(gamepad, Constants.GamepadR1);
    gamepadL1 = new JoystickButton(gamepad, Constants.GamepadL1);
    gamepadR3 = new JoystickButton(gamepad, Constants.GamepadR3);
    gamepadL3 = new JoystickButton(gamepad, Constants.GamepadL3);

    // This is all of the commands called by the joystick buttons - I like to
    // seperate gamepad and joystick for easily finding a command
    // Joystick Functions
    rBottom.whenPressed(m_drivetrainSubsystem::zeroGyroscope);
    rTrigger.whenPressed(m_drivetrainSubsystem::setPitchOffset);
    rInside.whenHeld(new AutoBalance());
    lBottom.whenHeld(new Align_With_Apriltag());


    // Not much to say about this one, it is just the gamepad version of the
    // joysticks above
    // Gamepad Functions
  }


  // Defines the deadband function. A deadband doesn't allow values under a
  // certain value to get passed to the motors to avoid them moving at the
  // slightest bump of a joystick.
  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  // This is a function that uses both the deadband function and squares the
  // value. To be honest not sure what the square is for, but I'm sure it is
  // required for the math regarding swerve drive angles.
  public static double modifyAxis(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Square the axis
    value = Math.copySign(value * value, value);

    return value;
  }


  // This function returns the name of the drivetrainSubsystem so that it can be
  // used by other commands and subsystems.
  public static DrivetrainSubsystem getDrivetrain() {
    return m_drivetrainSubsystem;
  }

  /*
  public static Arm getArm() {
    return m_arm;
  }
*/

public static VisionCamera getVisioncamera() {
  return m_visionCamera;
}


}
