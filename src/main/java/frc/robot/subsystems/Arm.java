// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  double kp = 0.5;
  double ki = 0.0;
  double kd = 0.0;
  CANSparkMax jointOne;
  PIDController pid;
  DutyCycleEncoder rEncoder;

  public Arm(int motorID, int encoderID) {
    jointOne = new CANSparkMax(motorID, MotorType.kBrushless);
    pid = new PIDController(kp, ki, kd);
    rEncoder = new DutyCycleEncoder(encoderID);

    jointOne.setIdleMode(IdleMode.kBrake);
    rEncoder.setDistancePerRotation(1);
  }

  public void armMoveTill(double val) {
    jointOne.set(pid.calculate(rEncoder.getAbsolutePosition(), val));
  }
  
  public void armStop() {
    jointOne.set(0.0);
  }

  @Override
  public void periodic() {
  }
}
