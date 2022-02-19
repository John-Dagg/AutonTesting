// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.io.Axis;
import frc.robot.subsystems.Drivetrain;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

public class RobotContainer {

  //Setting up auton files
  private String pathing = "CurvePath";

  private String trajectoryFile = "output/"+pathing+".wpilib.json";
  private Path trajectoryPath;
  private Trajectory trajectory;

  private String pathing1 = "Reverse1"; //Change this to change trajectory
  private String pathing2 = "Reverse2";

  private String trajectoryFile1 = "output/"+pathing1+".wpilib.json";
  private String trajectoryFile2 = "output/"+pathing2+".wpilib.json";
  private Path trajectoryPath1;
  private Path trajectoryPath2;
  private Trajectory trajectory1;
  private Trajectory trajectory2;

  private Trajectory fullTrajectory;

  //Subsystems
  Drivetrain mDrivetrain = new Drivetrain();

  RamseteCommand ramseteCommand;

  public RobotContainer() {

    configureButtonBindings();
    mDrivetrain.setDefaultCommand(new RunCommand(mDrivetrain::arcadeDrive, mDrivetrain));
    /*
    mDrivetrain.setDefaultCommand(new RunCommand(() -> mDrivetrain.arcadeDrive(
            Constants.driverController.getRawAxis(Axis.AxisID.LEFT_Y.getID()), Constants.driverController.getRawAxis(Axis.AxisID.RIGHT_X.getID())), mDrivetrain));

     */
  }

  private void configureButtonBindings() {

  }


  public Command getAutonomousCommand() {

    try {
      trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFile1);
      trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);

      trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFile2);
      trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);

      trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFile);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);


    } catch (IOException e){
      System.out.println("Couldn't find trajectory path");
      e.printStackTrace();
    }

    fullTrajectory = trajectory1.concatenate(trajectory2);

    var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                    new SimpleMotorFeedforward(
                            Constants.AutonDrivetrain.ks,
                            Constants.AutonDrivetrain.kv,
                            Constants.AutonDrivetrain.ka),
                    Constants.AutonDrivetrain.driveKinematics,
                    10);

    TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutonDrivetrain.maxVel,
                    Constants.AutonDrivetrain.maxAccel)
                    .setKinematics(Constants.AutonDrivetrain.driveKinematics)
                    .addConstraint(autoVoltageConstraint);


    Trajectory simpleTrajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
            new Pose2d(3, 0, new Rotation2d(0)), config);

    RamseteController mController = new RamseteController(Constants.AutonDrivetrain.ramseteB, Constants.AutonDrivetrain.ramseteZeta);

    mController.setEnabled(true);

    PIDController leftPID = new PIDController(Constants.AutonDrivetrain.kP, 0, 0);
    PIDController rightPID = new PIDController(Constants.AutonDrivetrain.kP, 0, 0);

    var table = NetworkTableInstance.getDefault().getTable("Troubleshooting");
    var leftReference = table.getEntry("Left Reference");
    var leftMeasurement = table.getEntry("Left Measurement");
    var rightReference = table.getEntry("Right Reference");
    var rightMeasurement = table.getEntry("Right Measurement");

    ramseteCommand = new RamseteCommand(
            trajectory,
            mDrivetrain::getPose,
            mController,
            new SimpleMotorFeedforward(
                    Constants.AutonDrivetrain.ks,
                    Constants.AutonDrivetrain.kv,
                    Constants.AutonDrivetrain.ka),
            Constants.AutonDrivetrain.driveKinematics,
            mDrivetrain::getWheelSpeeds,
            leftPID,
            rightPID,
            (leftVolts, rightVolts) -> {
              mDrivetrain.tankDriveVolts(leftVolts, rightVolts);

              SmartDashboard.putNumber("Left Reference", leftPID.getSetpoint());
              SmartDashboard.putNumber("Left Measurement", mDrivetrain.getWheelSpeeds().leftMetersPerSecond);

              SmartDashboard.putNumber("Right Reference", rightPID.getSetpoint());
              SmartDashboard.putNumber("Right Measurement", mDrivetrain.getWheelSpeeds().rightMetersPerSecond);

              leftMeasurement.setNumber(mDrivetrain.getWheelSpeeds().leftMetersPerSecond);
              leftReference.setNumber(leftPID.getSetpoint());

              rightMeasurement.setNumber(mDrivetrain.getWheelSpeeds().rightMetersPerSecond);
              rightReference.setNumber(rightPID.getSetpoint());
            },
            mDrivetrain);

    mDrivetrain.resetOdometry(trajectory.getInitialPose());

    return ramseteCommand.andThen(() -> mDrivetrain.tankDriveVolts(0,0));

  }
}
