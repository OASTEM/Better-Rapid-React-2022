package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Shoot;
import frc.robot.commands.DriveTrain.DriveStraight;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.DriveTrain.Driving;

public class OneBallAuto extends SequentialCommandGroup {

  public OneBallAuto(Intake intake, Shooter shooter, DriveTrain driveTrain) {
    addCommands(
        new WaitCommand(2),
        new Driving(driveTrain, -30, 0.04));//,
        //new Shoot(intake, shooter, -2600, 1000).withTimeout(5),
        //new DriveStraight(driveTrain, () -> -100).withTimeout(5));
  }
}
