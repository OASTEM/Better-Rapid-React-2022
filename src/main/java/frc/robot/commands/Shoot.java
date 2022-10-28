package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleBoard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight;

public class Shoot extends CommandBase {
    private Intake intake;
    private Shooter shooter;
    private DriveTrain driveTrain;
    Limelight limelight;
    private ShuffleBoard shuffleBoard;
    private boolean usingVision;
    private boolean isAuto;
    private int shooterVelocity;
    private int rollerVelocity;
    private IntSupplier suppliedVelocity;
    private IntSupplier suppliedRollerVelocity;
    private int pulseCounter;
    private int rpmCounter;
    public double visionAngle;
    private int turnCounter;
    private boolean shooterDebug = false;
    private double errorAngle;
    //private boolean shooting;

    // TELEOP VISION
    public Shoot(Intake intake, Shooter shooter, Limelight limelight, DriveTrain driveTrain) {
        addRequirements(intake, shooter, limelight, driveTrain);
        this.intake = intake;
        this.shooter = shooter;
        this.isAuto = false;
        this.limelight = limelight;
        this.usingVision = true;
        this.driveTrain = driveTrain;
    }

    // TELEOP DEBUG MODE
    public Shoot(Intake intake, Shooter shooter, Limelight limelight, DriveTrain driveTrain, boolean shooterDebug) {
        addRequirements(intake, shooter);
        this.intake = intake;
        this.shooter = shooter;
        this.limelight = limelight;
        this.driveTrain = driveTrain;
        this.isAuto = false;
        this.usingVision = true;
        this.shooterDebug = true;
    }

    // AUTO
    public Shoot(Intake intake, Shooter shooter, Limelight limelight, int shooterVelocity, int rollerVelocity, DriveTrain driveTrain) {
        //addRequirements(intake, shooter, limelight, driveTrain);
        this.intake = intake;
        this.shooter = shooter;
        this.isAuto = true;
        this.limelight = limelight;
        this.usingVision = false;
        this.driveTrain = driveTrain;
        this.shooterDebug = false;
        this.shooterVelocity = shooterVelocity;
        this.rollerVelocity = rollerVelocity;
    }

    @Override
    public void initialize() {
        visionAngle = limelight.getHorizontalAngle();
        
        errorAngle = Math.abs(visionAngle);
        // if (shooterDebug) {
        //     shooter.setPIDFront(shuffleBoard.getShooterPID());
        // }
        if (usingVision) {
            //shooting = false;
            visionAngle = limelight.getHorizontalAngle();
            turnCounter = 0;

        } else if (isAuto) {
            shooter.setVelocity(-shooterVelocity);
            shooter.setRollerVelocity(rollerVelocity);
        } else if (!usingVision) {
            shooter.setVelocity(suppliedVelocity.getAsInt());
            shooter.setRollerVelocity(suppliedRollerVelocity.getAsInt());
        } else {
            System.out.println("SOMETHING WENT REALLY WRONG UR STUPIDLY SCREWED");
        }
        pulseCounter = 0;
        rpmCounter = 0;
    }

    @Override
    public void execute() {
        if (isAuto == false) {
            int velocity = limelight.getVelocity();
            visionAngle = limelight.getHorizontalAngle();
            errorAngle = Math.abs(visionAngle);
            if ((turnCounter > 5 && Math.abs(errorAngle) < 3)) {
                driveTrain.tankDrive(0, 0);
                // System.out.println("Done turning!");
                System.out.println(limelight.debugRollerVelocity());
                SmartDashboard.putNumber("Distance in Inches", limelight.getDistance());
                if (shooterDebug){
                    shootWhenReady(-limelight.debugVelocity(), limelight.debugRollerVelocity());
                }
                else{
                    shootWhenReady(-velocity, velocity);
                }

            } else if (Math.abs(errorAngle) < 3) {
                turnCounter++;
                visionAngle = limelight.getHorizontalAngle();
                driveTrain.tankDrive(0, 0);
                System.out.println(velocity);
                if (!shooterDebug){
                shooter.setRollerVelocity(velocity);
                shooter.setVelocity(-velocity);
                }
                else{
                    SmartDashboard.putNumber("Distance in Inches", limelight.getDistance());
                    System.out.println(limelight.debugRollerVelocity());
                    shooter.setRollerVelocity(limelight.debugRollerVelocity());
                    shooter.setVelocity(-limelight.debugVelocity());
                }
            } else {
                double turnPower;
                if (Math.abs(visionAngle) <= 10 && Math.abs(visionAngle) >= 0) {
                    turnPower = Math.pow(errorAngle, 0.580667) * 0.0148639 + 0.0752756;
                } else {
                    turnPower = Math.pow(errorAngle, 0.706689) * 0.0152966 + 0.0550678;
                }
                turnPower = Math.min(turnPower, 0.3);
                if (visionAngle < 0) {
                    driveTrain.tankDrive(-turnPower, turnPower);
                } else {
                    driveTrain.tankDrive(turnPower, -turnPower);
                }

            }
        } else {
            // System.out.println("Placeholder");
            visionAngle = limelight.getHorizontalAngle();
            errorAngle = Math.abs(visionAngle);
            if ((turnCounter > 5 && Math.abs(errorAngle) < 3)) {
                driveTrain.tankDrive(0, 0);
                // System.out.println("Done turning!");
                SmartDashboard.putNumber("Distance in Inches", limelight.getDistance());
                shootWhenReady(-shooterVelocity, rollerVelocity);
            } else if (Math.abs(errorAngle) < 3) {
                turnCounter++;
                visionAngle = limelight.getHorizontalAngle();
                driveTrain.tankDrive(0, 0);
                // System.out.println(velocity);
                shooter.setRollerVelocity(rollerVelocity);
                shooter.setVelocity(-shooterVelocity);
            } else {
                double turnPower;
                if (Math.abs(visionAngle) <= 10 && Math.abs(visionAngle) >= 0) {
                    turnPower = Math.pow(errorAngle, 0.580667) * 0.0148639 + 0.0752756;
                } else {
                    turnPower = Math.pow(errorAngle, 0.706689) * 0.0152966 + 0.0550678;
                }
                turnPower = Math.min(turnPower, 0.3);
                if (visionAngle < 0) {
                    driveTrain.tankDrive(-turnPower, turnPower);
                } else {
                    driveTrain.tankDrive(turnPower, -turnPower);
                }

            }
        }
    }

    private void shootWhenReady(double velocity, double backVelocity) {
        // System.out.println("Velocity " + velocity);
        // System.out.println("backVelocity " + backVelocity);
        double error;
        double errorBack;
        error = Math.abs(velocity - shooter.getLeftVelocity());
        errorBack = Math.abs(backVelocity - shooter.getBackLeftVelocity());
        //System.out.println(error + " Not back**************");
        //System.out.println(errorBack + " Back******************");
        SmartDashboard.putNumber("Front Shooter LEFT RPM IMPORTANTE", shooter.getLeftVelocity());
        SmartDashboard.putNumber("Front Shooter RIGHT RPM IMPORTANTE", shooter.getRightVelocity());
        SmartDashboard.putNumber("Back Shooter RPM IMPORTANTE", shooter.getBackLeftVelocity());
        SmartDashboard.putNumber("Error", velocity - shooter.getLeftVelocity());
        SmartDashboard.putNumber("Back Error", backVelocity - shooter.getBackLeftVelocity());
        // System.out.println("Shooter motors getting ready");
        if (error <= Constants.Shooter.RPM_TOLERANCE && errorBack <= Constants.Shooter.RPM_TOLERANCE) {
            rpmCounter++;
            System.out.println("rpm counter++");
        }
        if (rpmCounter > 15) {
            if (pulseCounter < 10) {
                intake.intakeTopMotor(Constants.Shooter.PUSH_SPEED * -1);
                intake.intakeBottomMotor(Constants.Shooter.PUSH_SPEED);
                pulseCounter++;
            } else if (rpmCounter < 50) {
                intake.intakeTopMotor(0);
                intake.intakeBottomMotor(0);
                pulseCounter++;
            } else {
                intake.intakeTopMotor(0);
                intake.intakeBottomMotor(0);
                pulseCounter = 0;
                rpmCounter = 0;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.stop();
        shooter.stop();
        driveTrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
