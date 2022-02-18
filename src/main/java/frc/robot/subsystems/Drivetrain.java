package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

        mLeftMotors = new MotorControllerGroup(mLeftLeader, mLeftFollowerA, mLeftFollowerB);
        mRightMotors = new MotorControllerGroup(mRightLeader, mRightFollowerA, mLeftFollowerB);

        mRightMotors.setInverted(true);

        mDrive = new DifferentialDrive(mLeftMotors, mRightMotors);

        mLeftEncoder = mLeftLeader.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);
        mRightEncoder = mRightLeader.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, 4096);

        mPigeon = new Pigeon2(0);

        mDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(mPigeon.getYaw()));




    }
}
