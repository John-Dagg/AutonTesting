package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.Axis;
import frc.robot.util.MotorControllerFactory;

public class Drivetrain extends SubsystemBase {

    private CANSparkMax mLeftLeader, mLeftFollowerA, mLeftFollowerB, mRightLeader, mRightFollowerA, mRightFollowerB;
    private DoubleSolenoid mShifter;
    private RelativeEncoder mLeftEncoder, mRightEncoder;

    private MotorControllerGroup mLeftMotors, mRightMotors;
    private DifferentialDrive mDrive;
    private DifferentialDriveOdometry mDriveOdometry;

    private Pigeon2 mPigeon;

    public Drivetrain(){

        mLeftLeader = MotorControllerFactory.makeSparkMax(Constants.Drivetrain.leftLeaderPort);
        mLeftFollowerA = MotorControllerFactory.makeSparkMax(Constants.Drivetrain.leftFollowerAPort);
        mLeftFollowerB = MotorControllerFactory.makeSparkMax(Constants.Drivetrain.leftFollowerBPort);
        mRightLeader = MotorControllerFactory.makeSparkMax(Constants.Drivetrain.rightLeaderPort);
        mRightFollowerA = MotorControllerFactory.makeSparkMax(Constants.Drivetrain.rightFollowerAPort);
        mRightFollowerB = MotorControllerFactory.makeSparkMax(Constants.Drivetrain.rightFollowerBPort);

        mLeftLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mLeftFollowerA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mLeftFollowerB.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mRightLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mRightFollowerA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        mRightFollowerB.setIdleMode(CANSparkMax.IdleMode.kBrake);

        mLeftFollowerA.follow(mLeftLeader);
        mLeftFollowerB.follow(mLeftLeader);
        mRightFollowerA.follow(mRightLeader);
        mRightFollowerB.follow(mRightLeader);

        mRightLeader.setInverted(true);

        mLeftMotors = new MotorControllerGroup(mLeftLeader, mLeftFollowerA, mLeftFollowerB);
        mRightMotors = new MotorControllerGroup(mRightLeader, mRightFollowerA, mRightFollowerB);
//        mLeftMotors.setInverted(false); //Works
//        mRightMotors.setInverted(true); //Works


        mDrive = new DifferentialDrive(mLeftMotors, mRightMotors);

        mLeftEncoder = mLeftLeader.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
        mRightEncoder = mRightLeader.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);

        mPigeon = new Pigeon2(0);

        mDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(mPigeon.getYaw()));

    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(mLeftEncoder.getVelocity(), mRightEncoder.getVelocity());
    }

    public double getHeading(){
        return mPigeon.getYaw();
    }

    @Override
    public void periodic(){
        mDriveOdometry.update(Rotation2d.fromDegrees(mPigeon.getYaw()), mLeftEncoder.getPosition(), mRightEncoder.getPosition());
    }

    public Pose2d getPose(){
        return mDriveOdometry.getPoseMeters();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts){
        mLeftMotors.setVoltage(leftVolts);
        mRightMotors.setVoltage(rightVolts);
        mDrive.feed();
        printVelocity();
    }

    public void resetEncoders(){
        mLeftEncoder.setPosition(0);
        mRightEncoder.setPosition(0);
    }

    public void resetYaw(){
        mPigeon.setYaw(0);
    }

    public void arcadeDrive(){
        double throttle = deadband(Constants.driverController.getRawAxis(Axis.AxisID.LEFT_Y.getID()));
        double turn = deadband(Constants.driverController.getRawAxis(Axis.AxisID.RIGHT_X.getID()));

        double left = throttle - turn;
        double right = throttle + turn;

        mLeftLeader.set(left);
        mRightLeader.set(right);
    }

    public double deadband(double percentOutput){
        return Math.abs(percentOutput) > Constants.Drivetrain.deadband ? percentOutput : 0;
    }

    public void printDistance(){
        System.out.println("LEFT ENCODER DISTANCE: "+mLeftEncoder.getPosition());
        System.out.println("RIGHT ENCODER DISTANCE "+mRightEncoder.getPosition());

    }

    public void printVelocity(){
        System.out.println("LEFT ENCODER VELOCITY: "+mLeftEncoder.getVelocity());
        System.out.println("RIGHT ENCODER VELOCITY: "+mRightEncoder.getVelocity());
    }

    public void resetOdometry(Pose2d pose){
        resetEncoders();
        mDriveOdometry.resetPosition(pose, Rotation2d.fromDegrees(mPigeon.getYaw()));
    }


}
