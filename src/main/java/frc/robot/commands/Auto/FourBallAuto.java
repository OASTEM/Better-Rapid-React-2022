package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.Shoot;
import frc.robot.commands.DriveTrain.DriveStraight;
import frc.robot.commands.DriveTrain.TurnToAngle;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Shooter;

public class FourBallAuto extends SequentialCommandGroup {

  public FourBallAuto(Intake intake, Shooter shooter, DriveTrain driveTrain, NavX navX, Limelight limelight) {

    addCommands(
        new DriveStraight(driveTrain, 45).withTimeout(3).alongWith(
            new IntakeBalls(intake).withTimeout(3)),
        new TurnToAngle(driveTrain, navX, 165).withTimeout(2),
        new Shoot(intake, shooter, limelight, -3500, 1500, driveTrain).withTimeout(5),
        new TurnToAngle(driveTrain, navX, -150).withTimeout(2),
        new DriveStraight(driveTrain, 150).withTimeout(5).alongWith(
            new IntakeBalls(intake).withTimeout(5)),
        new DriveStraight(driveTrain, -150).withTimeout(4),
        new TurnToAngle(driveTrain, navX, 150).withTimeout(2),
        new Shoot(intake, shooter, limelight, -3500, 1500, driveTrain).withTimeout(5),
        new DriveStraight(driveTrain, -50).withTimeout(3)
    );
  }
}