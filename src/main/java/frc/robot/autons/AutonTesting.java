package frc.robot.autons;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class AutonTesting extends CommandBase {

    private Drivetrain mDrivetrain;
    DifferentialDriveVoltageConstraint autoVoltageConstraint;
    TrajectoryConfig config;
    Trajectory simpleAuton;
    RamseteCommand ramseteCommand;

    public AutonTesting(Drivetrain subsystem){
        mDrivetrain = subsystem;
        addRequirements(mDrivetrain);

        autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                        Constants.AutonDrivetrain.ks,
                        Constants.AutonDrivetrain.kv,
                        Constants.AutonDrivetrain.ka),
                Constants.AutonDrivetrain.driveKinematics, 10);

        config = new TrajectoryConfig(
                Constants.AutonDrivetrain.maxVel, Constants.AutonDrivetrain.maxAccel)
                .setKinematics(Constants.AutonDrivetrain.driveKinematics)
                .addConstraint(autoVoltageConstraint);

//        simpleAuton = TrajectoryGenerator.generateTrajectory(

        ramseteCommand = new RamseteCommand()

    }

    @Override
    public void initialize(){


    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean isFinished){

    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
