// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Drivetrain;

import java.io.IOException;
import java.nio.file.Path;

public class RobotContainer {

  //Setting up auton files
  private String pathing = "Circle2.0";

  private String trajectoryFile = "output/"+pathing+".wpilib.json";
  private Path trajectoryPath;
  private Trajectory trajectory;

  private String pathing1 = "5BallPath1"; //Change this to change trajectory
  private String pathing2 = "5BallPath2";
  private String pathing3 = "5BallPath3";

  private String trajectoryFile1 = "output/"+pathing1+".wpilib.json";
  private String trajectoryFile2 = "output/"+pathing2+".wpilib.json";
  private String trajectoryFile3 = "output/"+pathing3+".wpilib.json";
  private Path trajectoryPath1;
  private Path trajectoryPath2;
  private Path trajectoryPath3;
  private Trajectory trajectory1;
  private Trajectory trajectory2;
  private Trajectory trajectory3;

  private Trajectory fullTrajectory;

  //Subsystems
  Drivetrain mDrivetrain = new Drivetrain();

  RamseteCommand ramseteCommand;

  public RobotContainer() {

    configureButtonBindings();
    mDrivetrain.setDefaultCommand(new RunCommand(mDrivetrain::arcadeDrive, mDrivetrain));

  }

  private void configureButtonBindings() {

  }


  public Command getAutonomousCommand() {

    try {
      trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFile1);
      trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath1);

      trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFile2);
      trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath2);

      trajectoryPath3 = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFile3);
      trajectory3 = TrajectoryUtil.fromPathweaverJson(trajectoryPath3);

      trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFile);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);


    } catch (IOException e){
      System.out.println("Couldn't find trajectory path");
      e.printStackTrace();
    }

    fullTrajectory = trajectory1.concatenate(trajectory2).concatenate(trajectory3);

    RamseteController mController = new RamseteController(Constants.AutonDrivetrain.ramseteB, Constants.AutonDrivetrain.ramseteZeta);

    mController.setEnabled(true);

    PIDController leftPID = new PIDController(Constants.AutonDrivetrain.kP, 0, 0);
    PIDController rightPID = new PIDController(Constants.AutonDrivetrain.kP, 0, 0);

    ramseteCommand = new RamseteCommand(
            fullTrajectory,
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

            },
            mDrivetrain);

    mDrivetrain.resetOdometry(fullTrajectory.getInitialPose());

    return ramseteCommand.andThen(() -> mDrivetrain.tankDriveVolts(0,0));

  }
}
