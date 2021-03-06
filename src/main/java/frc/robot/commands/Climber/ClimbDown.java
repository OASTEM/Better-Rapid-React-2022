package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Constants;
import frc.robot.subsystems.Climber;

public class ClimbDown extends CommandBase {
  private final Climber climber;
  private boolean usingPID;
  private double leftEncoder;
  private double rightEncoder;

  public ClimbDown(Climber climber) {
    addRequirements(climber);
    this.climber = climber;
    this.usingPID = false;
  }

  public ClimbDown(Climber climber, double leftEncoder, double rightEncoder) {
    addRequirements(climber);
    this.climber = climber;
    this.usingPID = true;
    this.leftEncoder = leftEncoder;
    this.rightEncoder = rightEncoder;
  }

  @Override
  public void initialize() {
    if (usingPID){
      climber.selectProfile(Constants.Climber.DOWN.s);
      climber.setPositionLeft(leftEncoder);
      climber.setPositionRight(rightEncoder);
    } else {
      climber.climb(Constants.Climber.DOWN_SPEED);
    }
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    climber.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
