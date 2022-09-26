// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private final double cameraAngle = 0;
  private final int frameWidth = 960;
  private final int frameHeight = 720;
  // TODO Set the degree of the limelight degrees
  // TODO Set the height of the limelight inches
  // TODO Set the height of the goal inches

  private double limelightMountDegree = 2;
  private double limelightHeight = 2; 
  private double goalHeight = 2;  
  private double angleToGoalDegrees;
  private double angleToGoalInRadians;
  private double distance;

  private NetworkTableEntry tx; 
  private NetworkTableEntry ty; 
  private NetworkTableEntry ta;
  public double x;
  public double y;
  public double area;
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  public Limelight() {
    this.tx = table.getEntry("tx");
    this.ty = table.getEntry("ty");
    this.ta = table.getEntry("ta");
  }

  @Override
  public void periodic() {
    this.x = tx.getDouble(0.0);
    this.y = ty.getDouble(0.0);
    this.area = ta.getDouble(0.0);

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);

    angleToGoalDegrees = limelightMountDegree +  y;
    angleToGoalInRadians = angleToGoalDegrees * (Math.PI / 180.0);
    distance = (goalHeight - limelightHeight)/Math.tan(angleToGoalInRadians);


  }

  public double getDistance(){
    System.out.println(distance);
    return distance;
  }
  
  public double getFrontRPM(){
    return 1.0;
  }

  public double getBackRPM(){
    return 1.0;
  }
}