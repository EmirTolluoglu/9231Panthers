package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.sequence.IntakeSequence;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.DriverControlsSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import java.io.File;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


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
  private final IntakeSubsystem intakeSubsystem;
  private final SendableChooser<Command> autoChooser;
  private  final DriverControlsSubsystem driverControlsSubsystem;

  private UsbCamera cam;
  private UsbCamera cam2;
  
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public RobotContainer() 
  {

    cam=CameraServer.startAutomaticCapture(0);
    cam2=CameraServer.startAutomaticCapture(1);

    driveSubsystem= new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
    shooterSubsystem=ShooterSubsystem.getInstance();
    intakeSubsystem=IntakeSubsystem.getInstance();
    driverControlsSubsystem = DriverControlsSubsystem.getInstance();
    

    Command driveFieldOrientedDirectAngle = driveSubsystem.driveCommand2(
      () -> MathUtil.applyDeadband(driverController.getLeftY(), OperatorConstants.LEFTY_DEADBAND),
      () -> MathUtil.applyDeadband(driverController.getLeftX(), OperatorConstants.LEFTX_DEADBAND),
      () ->  MathUtil.applyDeadband(driverController.getRightX(), OperatorConstants.RIGHTX_DEADBAND));

    driveSubsystem.setDefaultCommand(driveFieldOrientedDirectAngle);
    driverControlsSubsystem.registerTriggers();
    NamedCommands.registerCommand("intakeSequence", new IntakeSequence());



    autoChooser = AutoBuilder.buildAutoChooser("yerden al");
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
