// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Intake.IntakePivot;
import frc.robot.commands.Intake.IntakeRoller;
import frc.robot.commands.Shooter.ShooterPivot;
import frc.robot.commands.Shooter.ShooterRoller;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.DriverControls;
import frc.robot.subsystems.ShooterSubsystem;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Swerve.FieldOrientedDrive;
import frc.robot.commands.Swerve.LockPods;
import frc.robot.commands.Swerve.ResetGyro;

public class RobotContainer {
  
public static final Swerve S_SWERVE = new Swerve();
  public static final DriverControls S_DRIVERCONTROLS = new DriverControls();

/*   private final ShooterSubsystem shooterSubsystem;
 */  

  private final SendableChooser<Command> autoChooser;
/*   private final ShooterSubsystem shooterSubsystem;
 */
  public RobotContainer() 
  {
    configureBindings();
    setDefaultCommands();

    NamedCommands.registerCommand("lockPods", new LockPods(S_SWERVE));
    NamedCommands.registerCommand("resetGyro", new ResetGyro(S_SWERVE));
    NamedCommands.registerCommand("mobilityPointDrive", S_SWERVE.autonDriveBackwards(5));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode", autoChooser);
/*     shooterSubsystem = new ShooterSubsystem(); // Initialize the shooterSubsystem
 */

  }
  private void setDefaultCommands(){
        CommandScheduler.getInstance().setDefaultCommand(S_SWERVE, new FieldOrientedDrive(S_SWERVE, S_DRIVERCONTROLS));

  }
  private void configureBindings() {

    S_DRIVERCONTROLS.registerTriggers(S_SWERVE);
  }
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
