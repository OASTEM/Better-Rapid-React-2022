package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.Shoot;
import frc.robot.commands.DriveTrain.DriveStraight;
import frc.robot.commands.DriveTrain.Driving;
import frc.robot.commands.DriveTrain.TurnToAngle;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Shooter;

public class TwoBallAuto extends SequentialCommandGroup {
  Intake intake;
  Shooter shooter;
  DriveTrain driveTrain;
  NavX navX;

  
  public TwoBallAuto(Intake intake, Shooter shooter, DriveTrain driveTrain, NavX navX, double angle, Limelight limelight) {
    addCommands(
      new Driving(driveTrain, -40, 0.014),
      new Shoot(intake, shooter, limelight, driveTrain).withTimeout(6),
        new TurnToAngle(driveTrain, navX, angle).withTimeout(2),
        new Driving(driveTrain, 40, 0.014).withTimeout(2).alongWith(new IntakeBalls(intake).withTimeout(2)),
        new Driving(driveTrain, -40, 0.014),
        new TurnToAngle(driveTrain, navX, -angle).withTimeout(2),
        new Shoot(intake, shooter, limelight, driveTrain).withTimeout(3),
        new DriveStraight(driveTrain, -50).withTimeout(3));
  }
} 
