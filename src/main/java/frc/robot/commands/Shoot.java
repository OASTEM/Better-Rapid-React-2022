package frc.robot.commands;

import java.util.function.IntSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.utils.Constants;
import frc.robot.utils.ShuffleBoard;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Limelight;

public class Shoot extends CommandBase {
    private Intake intake;
    private Shooter shooter;
    private DriveTrain driveTrain;
    private NavX navX;
    private Limelight limelight;
    private ShuffleBoard shuffleBoard;
    private boolean usingVision;
    private boolean isAuto;
    private int shooterVelocity;
    private int rollerVelocity;
    private IntSupplier suppliedVelocity;
    private IntSupplier suppliedRollerVelocity;
    private int pulseCounter;
    private int rpmCounter;
    private int visionFrontRPM;
    private int visionBackRPM;
    private double visionAngle;
    private int turnCounter;
    private boolean shooterDebug = false;
    private boolean shooting;

    // TELEOP VISION
    public Shoot(Intake intake, Shooter shooter, Limelight limelight, DriveTrain driveTrain, NavX navX) {
        addRequirements(intake, shooter, limelight, driveTrain);
        this.intake = intake;
        this.shooter = shooter;
        this.isAuto = false;
        this.limelight = limelight;
        this.usingVision = true;
        this.navX = navX;
        this.driveTrain = driveTrain;
    }

    // TELEOP DEBUG MODE
    public Shoot(Intake intake, Shooter shooter, IntSupplier suppliedVelocity, IntSupplier suppliedRollerVelocity, ShuffleBoard shuffleBoard) {
        addRequirements(intake, shooter);
        this.intake = intake;
        this.shooter = shooter;
        this.isAuto = false;
        this.usingVision = false;
        this.shooterDebug = true;
        this.suppliedVelocity = suppliedVelocity;
        this.suppliedRollerVelocity = suppliedRollerVelocity;
        this.shuffleBoard = shuffleBoard;
    }
    
    // AUTO
    public Shoot(Intake intake, Shooter shooter, int shooterVelocity, int rollerVelocity) {
        addRequirements(intake, shooter);
        this.intake = intake;
        this.shooter = shooter;
        this.isAuto = true;
        this.usingVision = false;
        this.shooterVelocity = shooterVelocity;
        this.rollerVelocity = rollerVelocity;
    }

    @Override
    public void initialize() {
        if (shooterDebug){
            shooter.setPIDFront(shuffleBoard.getShooterPID());
        }
        if (usingVision) {
            shooting = false;
            visionAngle = limelight.getHorizontalAngle();
            
            navX.reset();
            turnCounter = 0;

        } else if (isAuto) {
            shooter.setVelocity(shooterVelocity);
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
        if (usingVision) {
            double errorAngle = Math.abs(visionAngle - navX.getAngle());
            if (shooting){
                shootWhenReady(visionFrontRPM, visionBackRPM);
            } else if (turnCounter > 3 && Math.abs(errorAngle) < 1) {
                visionFrontRPM = (int) limelight.getFrontRPM();
                visionBackRPM = (int) limelight.getBackRPM();
                shooter.setVelocity(visionFrontRPM);
                shooter.setRollerVelocity(visionBackRPM);
                shooting = true;
                driveTrain.tankDrive(0, 0);
            } else if (Math.abs(errorAngle) < 1) {
                turnCounter ++;
                visionAngle = limelight.getHorizontalAngle();
                navX.reset();
                driveTrain.tankDrive(0, 0);

            } else {
                double turnPower;
                if (Math.abs(visionAngle)<=10 && Math.abs(visionAngle) >= 0){
                    turnPower = Math.pow(errorAngle, 0.580667)*0.0148639+0.0752756;
                }
                else{
                    turnPower = Math.pow(errorAngle,0.706689)*0.0152966+0.0550678;
                }
                Math.min(turnPower, 0.3);
                if ((visionAngle - navX.getAngle()) < 0){
                    driveTrain.tankDrive(-turnPower,turnPower);
                } else {
                    driveTrain.tankDrive(turnPower, -turnPower);
                }
            }
        } else {
            shootWhenReady(isAuto ? shooterVelocity : suppliedVelocity.getAsInt(),
                    isAuto ? rollerVelocity : suppliedRollerVelocity.getAsInt());
        }
    }

    private void shootWhenReady(double velocity, double backVelocity) {
        double error = Math.abs(velocity - shooter.getLeftVelocity());
        double errorBack = Math.abs(backVelocity - shooter.getBackLeftVelocity());
        SmartDashboard.putNumber("Front Shooter LEFT RPM IMPORTANTE", shooter.getLeftVelocity());
        SmartDashboard.putNumber("Front Shooter RIGHT RPM IMPORTANTE", shooter.getRightVelocity());
        SmartDashboard.putNumber("Back Shooter RPM IMPORTANTE", shooter.getBackLeftVelocity());
        if (error <= Constants.Shooter.RPM_TOLERANCE && errorBack <= Constants.Shooter.RPM_TOLERANCE) {
            rpmCounter++;
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
