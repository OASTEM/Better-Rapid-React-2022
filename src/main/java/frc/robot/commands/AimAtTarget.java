// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.DriveTrain;


public class AimAtTarget extends CommandBase {
  /** Creates a new AimAtTarget. */
  private Limelight limelight;
  private DriveTrain driveTrain;
  private NavX navX;
  private double visionAngle;
  private int turnCounter;
  private double errorAngle;



  public AimAtTarget(DriveTrain driveTrain, Limelight limelight, NavX navX) {
    addRequirements(driveTrain, limelight);
    this.driveTrain = driveTrain;
    this.limelight = limelight;
    this.navX = navX;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    visionAngle = limelight.getHorizontalAngle();
    navX.reset();
    errorAngle = Math.abs(visionAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Distance in Inches", limelight.getDistance());
    SmartDashboard.putNumber("Limelight Angle", limelight.getLimelightAngle());
    visionAngle = limelight.getHorizontalAngle();
    errorAngle = Math.abs(visionAngle);
    System.out.println("NavX " + navX.getAngle());
    System.out.println("Limelight " + limelight.getHorizontalAngle());
    System.out.println("Error " + errorAngle);
    System.out.println("Emily did this");
    if ((turnCounter > 5 && Math.abs(errorAngle) < 3)){
      driveTrain.tankDrive(0, 0);
      System.out.println("Done turning!");
    }
    else if (Math.abs(errorAngle) < 3){
      turnCounter ++;
      visionAngle = limelight.getHorizontalAngle();
      navX.reset();
      driveTrain.tankDrive(0, 0);
    } 
    else{
      double turnPower;
      if (Math.abs(visionAngle)<=10 && Math.abs(visionAngle) >= 0){
          turnPower = Math.pow(errorAngle, 0.580667)*0.0148639+0.0752756;
      }
      else{
          turnPower = Math.pow(errorAngle,0.706689)*0.0152966+0.0550678;
      }
      turnPower = Math.min(turnPower, 0.3);
      if (visionAngle < 0){
          driveTrain.tankDrive(-turnPower,turnPower);
      } else {
          driveTrain.tankDrive(turnPower, -turnPower);
      }

    }
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
