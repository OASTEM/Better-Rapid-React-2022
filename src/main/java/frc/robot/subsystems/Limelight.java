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

  private double limelightMountDegree = 34.3;


  //Test Values
  private double limelightHeight = 30; 
  private double goalHeight = 104;   

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


  }

  public double getDistance(){
    angleToGoalDegrees = limelightMountDegree +  ty.getDouble(0.0);
    angleToGoalInRadians = Math.toRadians(angleToGoalDegrees);
    distance = (goalHeight - limelightHeight)/Math.tan(angleToGoalInRadians);
    return distance;
  }

  public double getLimelightAngle(){
    double length = 119;
    // In inches adjacent
    double angle = (goalHeight - limelightHeight) / length;
    System.out.println("Y Value: " + ty.getDouble(0.0));
    angle = Math.atan(angle);
    angle = Math.abs(angle - Math.toRadians(ty.getDouble(0.0)));
    angle = Math.toDegrees(angle);
    return angle;
  }
  
  public double getFrontRPM()  {
    return 2700;
    //return 215685/y - 4550;
  }

  public double getBackRPM()  {
    return -2700;
    //return -107843/y + 2275;
  }

  public double getHorizontalAngle(){
    return x;
  }

  public double getVerticalAngle(){
    return y;
  }
}