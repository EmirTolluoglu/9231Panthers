// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.Intake.IntakePivot;
import frc.robot.commands.Intake.IntakeRoller;
import frc.robot.commands.Shooter.ShooterPivot;
import frc.robot.commands.Shooter.ShooterRoller;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  XboxController driverXbox = new XboxController(0);

  private final SwerveSubsystem driveSubsystem ;
  private final ShooterSubsystem shooterSubsystem;
  private final SendableChooser<Command> autoChooser;

  private UsbCamera cam;
  
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() 
  {
    // Configure the trigger bindings

    cam=CameraServer.startAutomaticCapture();

    driveSubsystem= new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
    shooterSubsystem=ShooterSubsystem.getInstance();
    
    Command driveFieldOrientedDirectAngle = driveSubsystem.driveCommand2(
      () -> MathUtil.applyDeadband(driverController.getLeftY()*0.75, OperatorConstants.LEFTY_DEADBAND),
      () -> MathUtil.applyDeadband(driverController.getLeftX()*0.75, OperatorConstants.LEFTX_DEADBAND),
      () ->  MathUtil.applyDeadband(driverController.getRawAxis(2)*0.75, OperatorConstants.RIGHTX_DEADBAND));

    driveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);
    
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser("Test");
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() 
  {
    new JoystickButton(driverXbox, 2)
        .whileTrue(new IntakeRoller(Constants.IntakeConstants.AMP_SHOOT_POWER));
    new JoystickButton(driverXbox, 3)
        .whileTrue(new IntakeRoller(-Constants.IntakeConstants.ROLLER_POWER));

    new JoystickButton(driverXbox, 4)
        .whileTrue(new IntakePivot(Constants.IntakeConstants.PIVOT_POWER));
    new JoystickButton(driverXbox, 1)
        .whileTrue(new IntakePivot(-Constants.IntakeConstants.PIVOT_POWER));

    new JoystickButton(driverXbox, 6)
        .whileTrue(new ShooterPivot(Constants.ShooterConstant.PIVOT_POWER));
    new JoystickButton(driverXbox, 5)
        .whileTrue(new ShooterPivot(-Constants.ShooterConstant.PIVOT_POWER));

    new JoystickButton(driverXbox, 7)
        .whileTrue(new ShooterRoller(Constants.ShooterConstant.ROLLER_POWER));

    new JoystickButton(driverXbox, 10)
        .whileTrue(new ClimberCmd(Constants.ClimberConstant.CLIMBER_POWER));
    new JoystickButton(driverXbox, 9)
        .whileTrue(new ClimberCmd(-Constants.ClimberConstant.CLIMBER_POWER));
    
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
