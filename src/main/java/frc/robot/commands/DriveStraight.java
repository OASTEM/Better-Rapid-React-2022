// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class DriveStraight extends CommandBase {
  /** Creates a new DriveStraight. */
  private final DriveTrain driveTrain; 
  private double goal; // put goal in as a constant 

  public DriveStraight(DriveTrain driveTrain, double goal ) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.goal = goal; 
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.resetEncoders();
    driveTrain.setLeftPID(Constants.SLOT_ID, Constants.kP, Constants.kI, Constants.kD); //make into constants
    driveTrain.setRightPID(Constants.SLOT_ID, Constants.kP, Constants.kI, Constants.kD);
    driveTrain.setPosition(goal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  
  @Override
  public void end(boolean interrupted) {
    driveTrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double left = driveTrain.getLeftEncoderCount();
    double right = driveTrain.getRightEncoderCount();

    return (left >= goal-1000) && (right >= goal-1000) && (right <= goal+1000) && (left <= goal+1000);
  }
}