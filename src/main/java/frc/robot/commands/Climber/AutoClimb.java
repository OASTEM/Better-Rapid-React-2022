package frc.robot.commands.Climber;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Pivots;
import frc.robot.utils.Constants;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class AutoClimb extends SequentialCommandGroup {

  public AutoClimb(Climber climber, Pivots pivots) {
    addRequirements(climber, pivots);
    addCommands(
        new InstantCommand(() -> pivots.setPositionPivots(-41, Constants.Pivot.FAST_PID.s)).withTimeout(2),
        new ClimbDown(climber, 1000, 1000).withTimeout(2)
            .andThen(new PivotRelative(pivots, 20, Constants.Pivot.FAST_PID.s).withTimeout(1)),
        new ClimbUp(climber, 55000, 55000, Constants.Climber.UP.s).withTimeout(0.5)
            .andThen(new PivotRelative(pivots, -10, Constants.Pivot.FAST_PID.s).withTimeout(0.5)),
        new PivotRelative(pivots, 90, Constants.Pivot.SLOW_PID.s).withTimeout(1.5)
            .alongWith(new ClimbUp(climber, Constants.Climber.TURBO.s).withTimeout(1.5)),
        new PivotRelative(pivots, 10, Constants.Pivot.FAST_PID.s).withTimeout(1),
        new PivotRelative(pivots, -25, Constants.Pivot.FAST_PID.s).withTimeout(1.5),
        new ClimbDown(climber, 190000, 190000).withTimeout(1),
        new PivotRelative(pivots, -95, Constants.Pivot.FAST_PID.s).withTimeout(1),
        new ClimbDown(climber, 100000, 100000).withTimeout(1.5),
        new InstantCommand(() -> pivots.setPositionPivots(-41, Constants.Pivot.FAST_PID.s)).withTimeout(1),
        new WaitCommand(2)
    );
  }
}
