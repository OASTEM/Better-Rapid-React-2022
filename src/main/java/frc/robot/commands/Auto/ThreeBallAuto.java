package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.Shoot;
import frc.robot.commands.DriveTrain.DriveStraight;
import frc.robot.commands.DriveTrain.TurnToAngle;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.Constants;

public class ThreeBallAuto extends SequentialCommandGroup {
  private Intake intake;
  private Shooter shooter;
  private DriveTrain driveTrain;
  private NavX navX;
  private Constants.ShuffleBoard.Auto direction;
  private double turn;

  public ThreeBallAuto(Intake intake, Shooter shooter, DriveTrain driveTrain, NavX navX,
      Constants.ShuffleBoard.Auto direction) {

    switch (direction) {
      case ThreeBall_1:
        turn = 1;
        break;
      case ThreeBall_2:
        turn = -1;
        break;
      case ThreeBall_3:
        turn = -1;
        break;

      default:
        break;
    }
    addCommands(
        new DriveStraight(driveTrain, () -> 45).withTimeout(3).alongWith(
            new IntakeBalls(intake).withTimeout(3)),
        new TurnToAngle(driveTrain, navX, 180).withTimeout(2),
        new DriveStraight(driveTrain, () -> 60).withTimeout(2),
        new Shoot(intake, shooter, -3000, -1200).withTimeout(5),
        new TurnToAngle(driveTrain, navX, -100 * turn).withTimeout(2),
        new DriveStraight(driveTrain, () -> 100).withTimeout(4).alongWith(
            new IntakeBalls(intake).withTimeout(4)),
        new DriveStraight(driveTrain, () -> -50).withTimeout(2),
        new TurnToAngle(driveTrain, navX, 120*turn).withTimeout(2),
        new Shoot(intake, shooter, -3000, -1200).withTimeout(5),
        new DriveStraight(driveTrain, () -> -100).withTimeout(3)
    );
  }
}