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
  private NetworkTableEntry tv;
  public double x;
  public double y;
  public double area;
  public double isTarget;
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  public boolean targets;

  public Limelight() {
    this.tx = table.getEntry("tx");
    this.ty = table.getEntry("ty");
    this.ta = table.getEntry("ta");
    this.tv = table.getEntry("tv");
    SmartDashboard.putNumber("Debug Velo Lime", 0);
    SmartDashboard.putNumber("Debug Roller Velo Lime", 0);
  }


  @Override
  public void periodic() {
    this.x = tx.getDouble(0.0);
    this.y = ty.getDouble(0.0);
    this.area = ta.getDouble(0.0);
    this.isTarget = tv.getDouble(0.0);
    if (isTarget == 0) {
      targets = false;
    } else {
      targets = true;
    }

    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putBoolean("Targets", targets);
    SmartDashboard.putNumber("Angle", getLimelightAngle());
    SmartDashboard.putNumber("Distance", getDistance());

  }
 
  /**
   * This function gets the distance.
   * 
   * Long paragraph description of it
   * @return Returns the distance from the ground hub.
   */
  public double getDistance(){
    angleToGoalDegrees = limelightMountDegree +  ty.getDouble(0.0);
    angleToGoalInRadians = Math.toRadians(angleToGoalDegrees);
    distance = (goalHeight - limelightHeight)/Math.tan(angleToGoalInRadians);
    return distance;
  }

  /**
   * Gets the angle based on the distance from the hub
   * 
   * 
   * @return Returns the angle from the equation
   */
  public double getLimelightAngle(){
    double length = 119;
    // In inches adjacent
    double angle = (goalHeight - limelightHeight) / length;
    // Gets the ratio of opp and adj
    angle = Math.atan(angle);
    // Takes this ratio and returns the angle
    angle = Math.abs(angle - Math.toRadians(ty.getDouble(0.0)));
    // Accounts for any off center
    angle = Math.toDegrees(angle);
    // Converts it to degrees
    return angle;
  }
  
  // Gets the x angle coordinate
  public double getHorizontalAngle(){
    return x;
  }

  // Gets the y angle coordinate
  public double getVerticalAngle(){
    return y;
  }

  /**
   * Gets the velocity off a line of best fit
   * 
   * Uses a line of best fit based off some data to calculate the velocity. It uses a cubic line of best fit. 
   * @return The velocity to set the shooter as. 
   */
  public int getVelocity(){
    double distance = getDistance();
    int velocity = (int) (0.00239721 * Math.pow(distance, 3) + -1.04519 * Math.pow(distance, 2) + (158.821 * distance) - 4534.88);
    if (velocity > 5370){
      velocity = 5370;
    }
    SmartDashboard.putNumber("Limelight Velocity", velocity);
    return velocity;
  }

  public int debugVelocity(){
    double velo = SmartDashboard.getNumber("Debug Velo Lime", 0);
    int newVelo = (int) velo;
    return newVelo;
  }

  public int debugRollerVelocity(){
    double velo = SmartDashboard.getNumber("Debug Roller Velo Lime", 0);
    int newVelo = (int) velo;
    return newVelo;
  }

}