package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;

public class Intake extends SubsystemBase {
  private CANSparkMax topMotor;
  private CANSparkMax bottomMotor;
  private CANSparkMax outsideMotor;
  private CANSparkMax raisingMotor;
  private SparkMaxPIDController raisingMotorPIDController;
  private RelativeEncoder raisingMotorEncoder;

  

  public Intake() {
    topMotor = new CANSparkMax(Constants.ID.INTAKE_TOP_SPARK, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(Constants.ID.INTAKE_BOTTOM_SPARK, MotorType.kBrushless);
    outsideMotor = new CANSparkMax(Constants.ID.INTAKE_OUTSIDE_SPARK, MotorType.kBrushless);
    raisingMotor = new CANSparkMax(Constants.ID.INTAKE_RAISING_SPARK, MotorType.kBrushless);

    raisingMotorPIDController = raisingMotor.getPIDController();
    raisingMotorEncoder = raisingMotor.getEncoder();

    raisingMotorPIDController.setP(Constants.Intake.PID.p);
    raisingMotorPIDController.setI(Constants.Intake.PID.i);
    raisingMotorPIDController.setD(Constants.Intake.PID.d);
    raisingMotorPIDController.setFF(Constants.Intake.PID.f);
  }

  public void intakeTopMotor(double speed) {
    topMotor.set(speed);
  }

  public void intakeBottomMotor(double speed) {
    bottomMotor.set(-speed);
  }

  public void intakeOutsideMotor(double speed) {
    outsideMotor.set(speed);
  }

  public void setRaisingMotorSpeed(double speed) {
    raisingMotor.set(speed);
  }

  public double getRaisingCurrent() {
    return Math.abs(raisingMotor.getOutputCurrent());
  }

  public void setRaisingMotorPosition(double position){
    raisingMotorPIDController.setReference(position, CANSparkMax.ControlType.kPosition);  
  }

  public void resetEncoders() {
    raisingMotorEncoder.setPosition(0);
  }

  public void ejectThroughIntake(int count){
    intakeTopMotor(-Constants.Intake.TOP_SPEED);
    intakeBottomMotor(-Constants.Intake.BOTTOM_SPEED);
  }

  public void stop() {
    topMotor.set(0);
    bottomMotor.set(0);
    outsideMotor.set(0);
  }

  @Override
  public void periodic() {
  }
}