// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;


public class RobotContainer {


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

//    Trajectory auton = 

    return null;

  }
}
