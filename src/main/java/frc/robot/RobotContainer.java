package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.Climber.AutoClimb;
import frc.robot.commands.Climber.ClimbDown;
import frc.robot.commands.Climber.ClimbUp;
import frc.robot.commands.Climber.PivotRelative;
import frc.robot.commands.DriveTrain.ArcadeDrive;
import frc.robot.commands.Calibration;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.Shoot;
import frc.robot.commands.Auto.OneBallAuto;
import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.ColorSensor;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Pivots;
import frc.robot.subsystems.Shooter;
import frc.robot.utils.Constants;
import frc.robot.utils.LogitechGamingPad;
import frc.robot.utils.ShuffleBoard;
import frc.robot.subsystems.Limelight;

// a member was here
public class RobotContainer {
  private final LogitechGamingPad drivePad = new LogitechGamingPad(0);
  private final LogitechGamingPad opPad = new LogitechGamingPad(1);

  private final DriveTrain driveTrain = new DriveTrain();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();
  private final Intake intake = new Intake();
  private final NavX navX = new NavX();
  private final Pivots pivots = new Pivots();
  private final Limelight limelight = new Limelight();
  // private final ColorSensor colorSensor = new ColorSensor();

  private final JoystickButton driveA = new JoystickButton(drivePad, 1);
  private final JoystickButton driveB = new JoystickButton(drivePad, 2);
  private final JoystickButton driveX = new JoystickButton(drivePad, 3);
  private final JoystickButton driveY = new JoystickButton(drivePad, 4);
  private final JoystickButton leftBumper = new JoystickButton(drivePad, 5);
  private final JoystickButton rightBumper = new JoystickButton(drivePad, 6);
  // private final JoystickButton driveBackButton = new
  // JoystickButton(drivePad,7);
  private final JoystickButton driveStartButton = new JoystickButton(drivePad, 8);

  private final JoystickButton opA = new JoystickButton(opPad, 1);
  private final JoystickButton opB = new JoystickButton(opPad, 2);
  private final JoystickButton opX = new JoystickButton(opPad, 3);
  private final JoystickButton opY = new JoystickButton(opPad, 4);
  // private final JoystickButton opLeftBumper = new JoystickButton(opPad, 5);
  private final JoystickButton opRightBumper = new JoystickButton(opPad, 6);
  // private final JoystickButton opBackButton = new JoystickButton(opPad, 7);
  private final JoystickButton opStartButton = new JoystickButton(opPad, 8);

  private final ShuffleBoard shuffleBoard = new ShuffleBoard(
      intake, shooter, driveTrain, navX, limelight);

  public RobotContainer() {
    driveTrain.setDefaultCommand(new ArcadeDrive(driveTrain, drivePad));
  }

  public void configureButtonBindings() {
    // primary driver
    driveTrain.setSlowMode(true);
    rightBumper.whileHeld(new IntakeBalls(intake));
    // rightBumper.whileHeld(new
    // InstantCommand(driveTrain::trueSlowMode)).whenReleased(new
    // InstantCommand(driveTrain::falseSlowMode));

    if (Constants.Shooter.DEBUG_MODE) {
      leftBumper.whileHeld(new Shoot(intake, shooter, limelight, driveTrain, true));
    } else {
      // leftBumper.whileHeld(new AimAtTarget(driveTrain, limelight, navX));
      leftBumper.whileHeld(new Shoot(intake, shooter, limelight, driveTrain));
    }

    // driveStartButton.whenPressed(new Calibration(climber, pivots, intake,
    // driveTrain));
    driveStartButton.whenPressed(new InstantCommand(driveTrain::climbingTrue));

    driveX.whileHeld(new ConditionalCommand(new AutoClimb(climber, pivots), new InstantCommand(), driveTrain::getClimbing));
    driveY.whileHeld(new ConditionalCommand(new ClimbUp(climber, Constants.Climber.UP.s), new InstantCommand(), driveTrain::getClimbing));
    // InstantCommand(), driveTrain::getClimbing));
    //leftBumper.whenPressed(new PivotRelative(pivots, 15, Constants.Pivot.FAST_PID.s));
    //driveB.whenPressed(new PivotRelative(pivots, 15, Constants.Pivot.FAST_PID.s));
    driveB.whenPressed(new PivotRelative(pivots, -90, Constants.Pivot.FAST_PID.s));
    driveA.whileHeld(new ConditionalCommand(new ClimbDown(climber), new InstantCommand(), driveTrain::getClimbing));

    //driveB.whenPressed(new InstantCommand(driveTrain::toggleSlowMode));
  }

  public void configureOPButtonBindings() {
    // secondary driver
    opY.whileHeld(new ClimbUp(climber, Constants.Climber.UP.s)); // maybe add slow
    opA.whileHeld(new ClimbDown(climber));
    opX.whileHeld(new PivotRelative(pivots, 10, Constants.Pivot.FAST_PID.s));
    opB.whileHeld(new PivotRelative(pivots, -10, Constants.Pivot.FAST_PID.s));
    opRightBumper.whileHeld(new IntakeBalls(intake));
    opStartButton.whenPressed(new Calibration(climber, pivots, intake, driveTrain));
    // opLeftBumper.whileHeld(new Shoot(intake, shooter, () ->
    // shuffleBoard.getShooterVelocity(), () -> shuffleBoard.getRollerVelocity(),
    // shuffleBoard).beforeStarting(new WaitCommand(1)).alongWith(new
    // DriveStraight(driveTrain, () -> shuffleBoard.getMoveBack())));
  }

  public Command getAutonomousCommand() {
    // return shuffleBoard.getAutonomousCommand();
    return new OneBallAuto(intake, shooter, driveTrain, limelight, navX);
    // return new Shoot(intake, shooter, limelight, driveTrain, navX);
  }

  public Command calibrate() {
    return new Calibration(climber, pivots, intake, driveTrain);
  }
}
