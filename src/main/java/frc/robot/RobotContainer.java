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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;

import java.io.IOException;



public class RobotContainer {

  Drivetrain mDrivetrain = new Drivetrain();

  Trajectory auton;
  RamseteCommand ramseteCommand;

  public RobotContainer() {

    configureButtonBindings();
  }

  private void configureButtonBindings() {}


  public Command getAutonomousCommand() {

    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                    Constants.AutonDrivetrain.ks,
                    Constants.AutonDrivetrain.kv,
                    Constants.AutonDrivetrain.ka),
            Constants.AutonDrivetrain.driveKinematics, 10);

    TrajectoryConfig config = new TrajectoryConfig(
            Constants.AutonDrivetrain.maxVel, Constants.AutonDrivetrain.maxAccel)
            .setKinematics(Constants.AutonDrivetrain.driveKinematics)
            .addConstraint(autoVoltageConstraint);

    try {
      auton = TrajectoryUtil.fromPathweaverJson(Constants.AutonDrivetrain.trajectoryPath);
    } catch (IOException e) {
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

//    mDrivetrain.reset(auton.getInitialPose());

    return ramseteCommand.andThen(() -> mDrivetrain.tankDriveVolts(0,0));

  }
}
