// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.LogitechGamingPad;


public class Driving extends CommandBase {
  DriveTrain driveTrain;

  double kP;
  double setpoint = 0;
  double leftError = 0;
  double rightError = 0;
  double leftSpeed = 0;
  double rightSpeed = 0;

  private Encoder encoder = new Encoder(0, 1, false, EncodingType.k4X);


  /** Creates a new Driving. */
  public Driving(DriveTrain driveTrain, double setpoint, double kP) {
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.setpoint = setpoint; 
    this.kP = kP;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetEncoders();
    leftError = setpoint - driveTrain.getInchesFromNativeUnits(driveTrain.getLeftEncoderCount());
    rightError = setpoint - driveTrain.getInchesFromNativeUnits(driveTrain.getRightEncoderCount());
    leftSpeed = kP * leftError;
    rightSpeed = kP * rightError;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    leftError = setpoint - driveTrain.getInchesFromNativeUnits(driveTrain.getLeftEncoderCount());
    rightError = setpoint - driveTrain.getInchesFromNativeUnits(driveTrain.getRightEncoderCount());
    leftSpeed = kP * leftError * 1.05;
    rightSpeed = kP * rightError;
    driveTrain.tankDrive(leftSpeed, rightSpeed);
    SmartDashboard.putNumber("Right Speed", rightSpeed);
    SmartDashboard.putNumber("Left Speed", leftSpeed);
    SmartDashboard.putNumber("Left Error", leftError);
    SmartDashboard.putNumber("Right Error", rightError);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
