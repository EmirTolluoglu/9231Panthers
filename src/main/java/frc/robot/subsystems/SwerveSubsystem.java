// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;

//import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;

import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveController;
import swervelib.SwerveDrive;

import edu.wpi.first.math.util.Units;

import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

  SwerveDrive swerveDrive;
  private static SwerveSubsystem instance;


  public SwerveSubsystem(File directory) 
  {
    try
    {
      swerveDrive = new SwerveParser(directory).createSwerveDrive(1);
    } 
    catch (Exception e)
    {
      throw new RuntimeException(e);
    }  
    instance=this;
    
        AutoBuilder.configureHolonomic(
            this::getPose,
            this::resetOdometry,
            this::getRobotOrientedVelocity,
            this::setChassisSpeed,
            new HolonomicPathFollowerConfig(
                new PIDConstants(Constants.kSwerveAutoPIDP, Constants.kSwerveAutoPIDI, Constants.kSwerveAutoPIDD),
                new PIDConstants(
                    swerveDrive.swerveController.config.headingPIDF.p,
                    swerveDrive.swerveController.config.headingPIDF.i,
                    swerveDrive.swerveController.config.headingPIDF.d),
                Constants.kMaxModuleSpeed,
                Units.feetToMeters(Constants.kDriveBaseRadius),
                new ReplanningConfig()
            ),
            this::shouldPathFlip,
            this
        );

  }

  public Command exampleMethodCommand() {
    return runOnce(
        () -> {
          
        });
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
                              DoubleSupplier headingY)
  {
        return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 1); // Smooth controll out
      double yInput = Math.pow(translationY.getAsDouble(), 1); // Smooth controll out
      // Make the robot move
      driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
                                                                      headingX.getAsDouble(),
                                                                      headingY.getAsDouble(),
                                                                      swerveDrive.getOdometryHeading().getRadians(),
                                                                      swerveDrive.getMaximumVelocity()));
    });
  }

  public Command driveCommand2(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier angularRotationX)
  {
    return run(() -> {
      // Make the robot move
      SmartDashboard.putNumber("Joystick", angularRotationX.getAsDouble());
      swerveDrive.drive(new Translation2d(-translationX.getAsDouble() * swerveDrive.getMaximumVelocity(),
                                          -translationY.getAsDouble() * swerveDrive.getMaximumVelocity()),
                        angularRotationX.getAsDouble() * swerveDrive.getMaximumAngularVelocity(),
                        true,
                        false);
    });
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }
 
  

   public void resetOdometry(Pose2d pose){
        swerveDrive.resetOdometry(pose);
    }

    public Pose2d getPose(){
        return swerveDrive.getPose();
    }

    public void setChassisSpeed(ChassisSpeeds velocity){
        swerveDrive.setChassisSpeeds(velocity);
    }
    public ChassisSpeeds getRobotOrientedVelocity(){
      return swerveDrive.getRobotVelocity();
  }
    public boolean shouldPathFlip(){
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

  @Override
  public void periodic() 
  {
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public SwerveSubsystem getInstance()
  {
      return instance;
  }

  public SwerveController getSwerveController(){
        return swerveDrive.swerveController;
    }


  public void driveFieldOriented(ChassisSpeeds velocity)
  {
    swerveDrive.driveFieldOriented(velocity);
  }


}
