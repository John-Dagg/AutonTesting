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
import frc.robot.subsystems.Drivetrain;

import java.io.IOException;
import java.nio.file.Path;

public class RobotContainer {

  //Setting up auton files
  private String pathing = "StraightPath"; //Change this to change trajectory

  private String trajectoryFile = "output/"+pathing+".wpilib.json";
  private Path trajectoryPath;
  private Trajectory trajectory;

  //Subsystems
  Drivetrain mDrivetrain = new Drivetrain();

  Trajectory auton;
  RamseteCommand ramseteCommand;

  public RobotContainer() {

    configureButtonBindings();
    mDrivetrain.setDefaultCommand(new RunCommand(mDrivetrain::arcadeDrive, mDrivetrain));
  }

  private void configureButtonBindings() {

  }


  public Command getAutonomousCommand() {

    try {
      trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryFile);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e){
      e.printStackTrace();
    }

    ramseteCommand = new RamseteCommand(
            auton,
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

    mDrivetrain.resetOdometry(auton.getInitialPose());

    return ramseteCommand.andThen(() -> mDrivetrain.tankDriveVolts(0,0));

  }
}
