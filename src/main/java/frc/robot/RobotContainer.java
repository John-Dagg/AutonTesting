// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.io.Axis;
import frc.robot.subsystems.Drivetrain;

import java.io.IOException;
import java.nio.file.Path;

public class RobotContainer {

  //Setting up auton files
  private String pathing = "Reverse1";

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

    var


    ramseteCommand = new RamseteCommand(
            trajectory,
            mDrivetrain::getPose,
            new RamseteController(Constants.AutonDrivetrain.ramseteB, Constants.AutonDrivetrain.ramseteZeta),
            new SimpleMotorFeedforward(
                    Constants.AutonDrivetrain.ks,
                    Constants.AutonDrivetrain.kv,
                    Constants.AutonDrivetrain.ka),
            Constants.AutonDrivetrain.driveKinematics,
            mDrivetrain::getWheelSpeeds,
            new PIDController(Constants.AutonDrivetrain.kP, 0 ,0),
            new PIDController(Constants.AutonDrivetrain.kP, 0 ,0),
            mDrivetrain::tankDriveVolts,
            mDrivetrain);

    mDrivetrain.resetOdometry(trajectory.getInitialPose());

    return ramseteCommand.andThen(() -> mDrivetrain.tankDriveVolts(0,0));

  }
}
