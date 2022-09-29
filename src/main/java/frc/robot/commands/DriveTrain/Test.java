// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Limelight;

public class Test extends CommandBase {
  /** Creates a new Test. */
  // Timer timer;
  DriveTrain driveTrain;
  Limelight limelight;
  double goal;
  double error;

  public Test(DriveTrain driveTrain, Limelight limelight) {
    addRequirements(driveTrain, limelight);
    this.driveTrain = driveTrain;
    this.limelight = limelight; 
    goal = 95000;
    // timer = new Timer();
  }

  @Override
  public void initialize() {
    // timer.reset();
    driveTrain.resetEncoders();
    // timer.start();

  }

  @Override
  public void execute() {
    SmartDashboard.putNumber("Horizontal Angle: ", limelight.getHorizontalAngle());
    SmartDashboard.putNumber("Vertical Angle: ", limelight.getVerticalAngle());
    // error = goal - driveTrain.getLeftEncoderCount();
    // driveTrain.arcadeDrive(0, (-error / goal) * .4);
  }

  @Override
  public void end(boolean interrupted) {
    // driveTrain.stop();
  }

  @Override
  public boolean isFinished() {
    // return timer.get() > 1;
    return false;
    // return driveTrain.getLeftEncoderCount() > goal;
  }
}
