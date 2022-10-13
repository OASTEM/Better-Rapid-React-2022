package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.commands.Auto.OneBallAuto;
import frc.robot.commands.Auto.FourBallAuto;
import frc.robot.commands.Auto.TwoBallAuto;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Shooter;

public class ShuffleBoard {
  private final SendableChooser<Constants.ShuffleBoard.Auto> autoChoose = new SendableChooser<Constants.ShuffleBoard.Auto>();

  private final SendableChooser<Shots> shooterLocationChooser = new SendableChooser<Shots>();

  private int shooterRPM = Constants.Shooter.FENDER.getFrontRPM();
  private int rollerRpm = Constants.Shooter.FENDER.getBackRPM();
  private PID shooterPID = Constants.Shooter.PID;
  private PID driveTrainPID = Constants.DriveTrain.PID;

  private Intake intake;
  private Shooter shooter;
  private DriveTrain driveTrain;
  private NavX navX;
  private Limelight limelight;

  public ShuffleBoard(Intake intake, Shooter shooter, DriveTrain driveTrain, NavX navX, Limelight limelight) {
    this.intake = intake;
    this.shooter = shooter;
    this.driveTrain = driveTrain;
    this.navX = navX;

    autoChoose.setDefaultOption("One Ball", Constants.ShuffleBoard.Auto.OneBall);
    autoChoose.addOption("Two Ball 1", Constants.ShuffleBoard.Auto.TwoBall_1);
    autoChoose.addOption("Two Ball 2", Constants.ShuffleBoard.Auto.TwoBall_2);
    autoChoose.addOption("Two Ball 3", Constants.ShuffleBoard.Auto.TwoBall_3);
    autoChoose.addOption("Four Ball 2", Constants.ShuffleBoard.Auto.FourBall);

    SmartDashboard.putData("AUTO MODE", autoChoose);

    shooterLocationChooser.setDefaultOption("Fender Shooting", Constants.Shooter.FENDER);
    shooterLocationChooser.addOption("Low Fender Shooting", Constants.Shooter.FENDER_LOW);
    shooterLocationChooser.addOption("Launchpad Shooting", Constants.Shooter.LAUNCHPAD);

    SmartDashboard.putData("Shooter Location", shooterLocationChooser);

    if (Constants.Shooter.DEBUG_MODE) {
      SmartDashboard.putNumber("Shooter Input RPM", shooterRPM);
      SmartDashboard.putNumber("Back Input RPM", rollerRpm);
      SmartDashboard.putNumber("Shooter P", shooterPID.p);
      SmartDashboard.putNumber("Shooter I", shooterPID.i);
      SmartDashboard.putNumber("Shooter D", shooterPID.d);
      SmartDashboard.putNumber("Shooter F", shooterPID.f);
    }

    if (Constants.DriveTrain.DEBUG) {
      SmartDashboard.putNumber("DriveTrain P", driveTrainPID.p);
      SmartDashboard.putNumber("DriveTrain P", driveTrainPID.p);
      SmartDashboard.putNumber("DriveTrain I", driveTrainPID.i);
      SmartDashboard.putNumber("DriveTrain D", driveTrainPID.d);
      SmartDashboard.putNumber("DriveTrain F", driveTrainPID.f);

    }

  }

  public Command getAutonomousCommand() {
    switch (autoChoose.getSelected()) {
      case OneBall:
        return new OneBallAuto(intake, shooter, driveTrain, limelight, navX);
      case TwoBall_1:
        return new TwoBallAuto(intake, shooter, driveTrain, navX, 160, limelight);
      case TwoBall_2:
        return new TwoBallAuto(intake, shooter, driveTrain, navX, -170, limelight);
      case TwoBall_3:
        return new TwoBallAuto(intake, shooter, driveTrain, navX, 180, limelight);
      case FourBall:
        return new FourBallAuto(intake, shooter, driveTrain, navX, limelight);
      default:
        return new OneBallAuto(intake, shooter, driveTrain, limelight, navX);

    }
  }

  public PID getShooterPID() {
    shooterPID.updatePID(
        SmartDashboard.getNumber("Shooter P", shooterPID.p),
        SmartDashboard.getNumber("Shooter I", shooterPID.i),
        SmartDashboard.getNumber("Shooter D", shooterPID.d),
        SmartDashboard.getNumber("Shooter F", shooterPID.f));
    return shooterPID;
  }

  public PID getDriveTrainPID() {
    driveTrainPID.updatePID(
        SmartDashboard.getNumber("DriveTrain P", driveTrainPID.p),
        SmartDashboard.getNumber("DriveTrain I", driveTrainPID.i),
        SmartDashboard.getNumber("DriveTrain D", driveTrainPID.d),
        SmartDashboard.getNumber("DriveTrain F", driveTrainPID.f));
    return driveTrainPID;
  }

  public int getShooterVelocity() {
    return shooterLocationChooser.getSelected().getFrontRPM();
  }

  public int getRollerVelocity() {
    return shooterLocationChooser.getSelected().getBackRPM();
  }

  public double getMoveBack() {
    return shooterLocationChooser.getSelected().getDistance();
  }

  public int getShoot() {
    return (int) SmartDashboard.getNumber("Shooter Input RPM", 0);
  }

  public int getRoller() {
    return (int) SmartDashboard.getNumber("Back Input RPM", 0);
  }

  public boolean getDebugMode() {
    return SmartDashboard.getBoolean("Debug Mode", false);
  }

  public static int getTeamAlliance() {
    return (int) NetworkTableInstance.getDefault().getTable("FMSInfo").getEntry("key").getDouble(-1);
    // -1 no team found 0 red 1 blue
  }
}
