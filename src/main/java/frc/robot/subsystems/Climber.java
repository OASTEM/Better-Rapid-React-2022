package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.PID;

public class Climber extends SubsystemBase {

  public TalonFX leftMotor;
  public TalonFX rightMotor;

  public Climber() {
    leftMotor = new TalonFX(Constants.ID.CLIMBER_LEFT);
    rightMotor = new TalonFX(Constants.ID.CLIMBER_RIGHT);

    rightMotor.setInverted(true);

    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);

    leftMotor.configPeakOutputForward(1);
    leftMotor.configPeakOutputReverse(-1);
    rightMotor.configPeakOutputForward(1);
    rightMotor.configPeakOutputReverse(-1);
    
    initPIDController(leftMotor, rightMotor, Constants.Climber.DOWN);
    initPIDController(leftMotor, rightMotor, Constants.Climber.UP);
    initPIDController(leftMotor, rightMotor, Constants.Climber.TURBO); 
  }

  private void initPIDController(TalonFX controller1, TalonFX controller2, PID PID) {
    controller1.config_kP(PID.s, PID.p);
    controller1.config_kI(PID.s, PID.i);
    controller1.config_kD(PID.s, PID.d);
    controller1.config_kF(PID.s, PID.f);

    controller2.config_kP(PID.s, PID.p);
    controller2.config_kI(PID.s, PID.i);
    controller2.config_kD(PID.s, PID.d);
    controller2.config_kF(PID.s, PID.f);

  }

  public void selectProfile(int id) {
    leftMotor.selectProfileSlot(id, 0);
    rightMotor.selectProfileSlot(id, 0);
  }

  public void climb(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
    rightMotor.set(ControlMode.PercentOutput, speed);
  }

  public void climbLeft(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
  }

  public void climbRight(double speed) {
    rightMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setPosition(double encoder) {
    leftMotor.set(ControlMode.Position, encoder);
    rightMotor.set(ControlMode.Position, encoder);
  }

  public void setPositionLeft(double encoder) {
    leftMotor.set(ControlMode.Position, encoder);
  }

  public void setPositionRight(double encoder) {
    rightMotor.set(ControlMode.Position, encoder);
  }

  public void resetEncoders() {
    leftMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
    rightMotor.getSensorCollection().setIntegratedSensorPosition(0, 0);
  }

  public double getLeftEncoderCount() {
    return leftMotor.getSensorCollection().getIntegratedSensorPosition();
  }

  public double getRightEncoderCount() {
    return rightMotor.getSensorCollection().getIntegratedSensorPosition();
  }

  public double getCurrentLeft() {
    return Math.abs(leftMotor.getStatorCurrent());
  }

  public double getCurrentRight() {
    return Math.abs(rightMotor.getStatorCurrent());
  }

  public void printEncoders() {
    System.out.println(leftMotor.getDeviceID() + ": " + getLeftEncoderCount());
    System.out.println(rightMotor.getDeviceID() + ": " + getRightEncoderCount());
  }

  public void stop() {
    leftMotor.set(ControlMode.PercentOutput, 0.0);
    rightMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public double getVelocityLeft() {
    return leftMotor.getSelectedSensorVelocity();
  }

  public double getVelocityRight() {
    return rightMotor.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
    //System.out.println("right encoder: " + getRightEncoderCount());
  }
}