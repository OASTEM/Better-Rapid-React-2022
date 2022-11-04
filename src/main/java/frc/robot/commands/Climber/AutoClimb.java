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
    //Positive pivots go towards the shooter / down
    //Negative pivots go up 
    addCommands(
        new InstantCommand(() -> pivots.setPositionPivots(-41, Constants.Pivot.FAST_PID.s)).withTimeout(2),
        new ClimbDown(climber, 1000, 1000).withTimeout(2)
            .andThen(new PivotRelative(pivots, 15, Constants.Pivot.FAST_PID.s).withTimeout(1)),
        // .andThen(new PivotRelative(pivots, 20,
        // Constants.Pivot.FAST_PID.s).withTimeout(1)),
        new ClimbUp(climber, 55000, 55000, Constants.Climber.UP.s).withTimeout(2)
            .alongWith(new InstantCommand(() -> pivots.setPositionPivots(-37, Constants.Pivot.FAST_PID.s)))
            .withTimeout(2)
            .andThen(new PivotRelative(pivots, -15, Constants.Pivot.FAST_PID.s).withTimeout(1)),
        new PivotRelative(pivots, 90, Constants.Pivot.SLOW_PID.s).withTimeout(1.2)
            .alongWith(new ClimbUp(climber, Constants.Climber.TURBO.s).withTimeout(1.2)),
        // new WaitCommand(1),
        new PivotRelative(pivots, -25, Constants.Pivot.MODERATE_PID.s).withTimeout(1.5),  //-25
        new PivotRelative(pivots, 10, Constants.Pivot.FAST_PID.s).withTimeout(.7),
        new PivotRelative(pivots, -20, Constants.Pivot.MODERATE_PID.s).withTimeout(1.5),
        // new WaitCommand(1),
        new ClimbDown(climber, 190000, 190000).withTimeout(1),
        new WaitCommand(1),
        new PivotRelative(pivots, -95, Constants.Pivot.FAST_PID.s).withTimeout(1),
        new ClimbDown(climber, 100000, 100000).withTimeout(1.5),
        new InstantCommand(() -> pivots.setPositionPivots(-41, Constants.Pivot.FAST_PID.s)).withTimeout(1),
        new WaitCommand(2));
  }
}
