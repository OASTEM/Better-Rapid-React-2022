package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.Shoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.DriveTrain.Driving;
import frc.robot.commands.DriveTrain.TurnToAngle;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.NavX;

public class OneBallAuto extends SequentialCommandGroup {

  public OneBallAuto(Intake intake, Shooter shooter, DriveTrain driveTrain, Limelight limelight, NavX navX) {
    addCommands(
        new WaitCommand(2),
        //new Driving(driveTrain, 10, 0.014).withTimeout(2),//,
        new Driving(driveTrain, -40, 0.014),
        new Shoot(intake, shooter, limelight, driveTrain).withTimeout(5),
        new IntakeBalls(intake).withTimeout(0.5),
        new Shoot(intake, shooter, limelight, driveTrain).withTimeout(7)
        // new Shoot(intake, shooter, limelight, driveTrain).withTimeout(4),
        // new Driving(driveTrain, -40, 0.014),
        );
        //new Shoot(intake, shooter, limelight, 2000, 2000, driveTrain, navX).withTimeout(5));
        //new Shoot(intake, shooter, limelight, driveTrain, navX).withTimeout(5));
        // new Driving(driveTrain, 10, 0.014).withTimeout(2));
        //new Shoot(intake, shooter, -2600, 1000).withTimeout(5),
        //new DriveStraight(driveTrain, () -> -100).withTimeout(5));
  }
}
