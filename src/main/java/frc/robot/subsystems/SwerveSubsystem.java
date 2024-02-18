// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
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
    
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
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

  public void drive(Translation2d translation, double rotation, boolean fieldRelative)
  {
    swerveDrive.drive(translation,
                      rotation,
                      fieldRelative,
                      false); // Open loop is disabled since it shouldn't be used most of the time.
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
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
