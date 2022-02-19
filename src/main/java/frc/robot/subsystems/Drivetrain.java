package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private double positionConversion = Math.PI * Units.inchesToMeters(6) * (double)1/7; //Converts a rotation of the axle with the encoder to meters per rotation of the drivetrain
    private double velocityConversion = Math.PI * Units.inchesToMeters(6) * (double)1/7 / 60; //Converts rpms to meters per second

    private double mLeftVolts, mRightVolts;

    public Drivetrain(){

        mLeftLeader = MotorControllerFactory.makeSparkMax(Constants.Drivetrain.leftLeaderPort);
        mLeftFollowerA = MotorControllerFactory.makeSparkMax(Constants.Drivetrain.leftFollowerAPort);
        mLeftFollowerB = MotorControllerFactory.makeSparkMax(Constants.Drivetrain.leftFollowerBPort);
        mRightLeader = MotorControllerFactory.makeSparkMax(Constants.Drivetrain.rightLeaderPort);
        mRightFollowerA = MotorControllerFactory.makeSparkMax(Constants.Drivetrain.rightFollowerAPort);
        mRightFollowerB = MotorControllerFactory.makeSparkMax(Constants.Drivetrain.rightFollowerBPort);
/*
        mLeftFollowerA.follow(mLeftLeader);
        mLeftFollowerB.follow(mLeftLeader);
        mRightFollowerA.follow(mRightLeader);
        mRightFollowerB.follow(mRightLeader);

        mRightLeader.setInverted(true);
        mRightFollowerA.setInverted(true);
        mRightFollowerB.setInverted(true);

 */

        mLeftLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mLeftFollowerA.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mLeftFollowerB.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mRightLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mRightFollowerA.setIdleMode(CANSparkMax.IdleMode.kCoast);
        mRightFollowerB.setIdleMode(CANSparkMax.IdleMode.kCoast);

        mLeftMotors = new MotorControllerGroup(mLeftLeader, mLeftFollowerA, mLeftFollowerB);
        mRightMotors = new MotorControllerGroup(mRightLeader, mRightFollowerA, mRightFollowerB);
        mLeftMotors.setInverted(true);
        mRightMotors.setInverted(false);

        mDrive = new DifferentialDrive(mLeftMotors, mRightMotors);

        mLeftEncoder = mLeftLeader.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
        mRightEncoder = mRightLeader.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
        mLeftEncoder.setInverted(true);
        mRightEncoder.setInverted(false);


        mPigeon = new Pigeon2(0);

        mDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(mPigeon.getYaw()));

        resetEncoders();
        resetYaw();

    }

    public void arcadeDrive(){
        double throttle = deadband(Constants.driverController.getRawAxis(Axis.AxisID.LEFT_Y.getID()));
        double turn = deadband(Constants.driverController.getRawAxis(Axis.AxisID.RIGHT_X.getID()));
/*
        double left = throttle - turn;
        double right = throttle + turn;

        mLeftLeader.set(left);
        mRightLeader.set(right);

        mDrive.feed();
*/
        mDrive.arcadeDrive(-throttle, turn);

    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        /*
        if (Math.abs(mLeftEncoder.getVelocity()) > 0) {
            System.out.println("LEFT METERS PER SECOND: " + mLeftEncoder.getVelocity() * velocityConversion + " | RIGHT METERS PER SECOND: " + mRightEncoder.getVelocity() * velocityConversion);
        }
         */
        return new DifferentialDriveWheelSpeeds(mLeftEncoder.getVelocity() * velocityConversion, mRightEncoder.getVelocity() * velocityConversion);
    }

    public double getHeading(){
        if (Math.abs(mPigeon.getYaw()) > 360){
            mPigeon.setYaw(0);
        } mPigeon.getYaw();

        return mPigeon.getYaw();
    }

    @Override
    public void periodic(){
        mDriveOdometry.update(Rotation2d.fromDegrees(mPigeon.getYaw()), mLeftEncoder.getPosition() * positionConversion, mRightEncoder.getPosition() * positionConversion);

//        System.out.println("Yaw: " + getHeading());

        if (Math.abs(mLeftVolts) > 0.0) {
//            System.out.println("LEFT VOLTS: " + mLeftVolts + " | RIGHT VOLTS: " + mRightVolts);
        }

//        printDistance();
//        printVelocity();

    }

    public Pose2d getPose(){
        return mDriveOdometry.getPoseMeters();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts){
        mLeftVolts = leftVolts / 12;
        mRightVolts = rightVolts / 12;

        mLeftMotors.setVoltage(mLeftVolts);
        mRightMotors.setVoltage(mRightVolts);

        mDrive.feed();
    }

    public void resetEncoders(){
        mLeftEncoder.setPosition(0);
        mRightEncoder.setPosition(0);
    }

    public void resetYaw(){
        mPigeon.setYaw(0);
    }

    public double deadband(double percentOutput){
        return Math.abs(percentOutput) > Constants.Drivetrain.deadband ? percentOutput : 0;
    }

    public void printDistance(){
        System.out.println("LEFT METERS: " + (mLeftEncoder.getPosition() * positionConversion) + " | RIGHT METERS: " + (mRightEncoder.getPosition() * positionConversion));
    }

    public void printVelocity(){
        if (Math.abs(mLeftEncoder.getVelocity()) > 0) {
            System.out.println("LEFT METERS PER SECOND: " + mLeftEncoder.getVelocity() * velocityConversion + " | RIGHT METERS PER SECOND: " + mRightEncoder.getVelocity() * velocityConversion);
        }
    }

    public void resetOdometry(Pose2d pose){
        resetEncoders();
        mDriveOdometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }


}
