package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.Intake.IntakePivot;
import frc.robot.commands.Intake.IntakeRoller;
import frc.robot.commands.Shooter.ShooterPivot;
import frc.robot.commands.Shooter.ShooterRoller;

public class DriverControlsSubsystem extends SubsystemBase{
    
    static DriverControlsSubsystem instance;
    private XboxController driverController;
 

    public DriverControlsSubsystem(){
        driverController = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
    }

    public XboxController getDriverController(){
        return driverController;
    }

    /*Intake roller */
    public boolean IntakeRollerIn(){
        return driverController.getBButton();
    }
    
    public boolean IntakeRollerOut(){
        return driverController.getAButton();
    }


    /*Intake pivot */
    public boolean IntakePivotPositive(){
        return driverController.getYButton();
    }
    
    public boolean IntakePivotNegative(){
        return driverController.getXButton();
    }

    /*Shooter pivot */
     public boolean ShooterPivotPositive(){
        return driverController.getRightBumper();
    }
     public boolean ShooterPivotNegative(){
        return driverController.getLeftBumper();
    }

    /*Shooter roller */
    public boolean ShooterRoller(){
        return driverController.getLeftTriggerAxis() > 0;
    }

    /*Climber */
    public boolean ClimberPositive(){
        return driverController.getBackButton();
    }
    public boolean ClimberNegative(){
        return driverController.getStartButton();
    }


    public void setRumble(double speed){
        driverController.setRumble(RumbleType.kBothRumble, speed);
    }


    public void registerTriggers(){

    // Intake
   new Trigger(this::IntakePivotPositive).whileTrue(new IntakeRoller(Constants.IntakeConstants.AMP_SHOOT_POWER));
    new Trigger(this::IntakePivotNegative).whileTrue(new IntakeRoller(-Constants.IntakeConstants.ROLLER_POWER));


    // Intake
    new Trigger(this::IntakePivotPositive).whileTrue(new IntakePivot(Constants.IntakeConstants.PIVOT_POWER));
    new Trigger(this::IntakePivotNegative).whileTrue(new IntakePivot(-Constants.IntakeConstants.PIVOT_POWER));


    // Shooter
    new Trigger(this::ShooterPivotPositive).whileTrue(new ShooterPivot(Constants.ShooterConstant.PIVOT_POWER));
    new Trigger(this::ShooterPivotNegative).whileTrue(new ShooterPivot(-Constants.ShooterConstant.PIVOT_POWER));

    // Shooter
    new Trigger(this::ShooterRoller).whileTrue(new ShooterRoller(Constants.ShooterConstant.ROLLER_POWER));

    // Climber
    new Trigger(this::ClimberPositive).whileTrue(new ClimberCmd(Constants.ClimberConstant.CLIMBER_POWER));
    new Trigger(this::ClimberNegative).whileTrue(new ClimberCmd(-Constants.ClimberConstant.CLIMBER_POWER));


    }
    public static DriverControlsSubsystem getInstance()
    {
        if(instance==null)
        {
            instance=new DriverControlsSubsystem();
        }
        return instance;
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("TIMER", DriverStation.getMatchTime());

        if(DriverStation.getMatchTime() < 20){
            setRumble(1);
        }
        else if(DriverStation.getMatchTime() < 10){
            setRumble(1);
        }
    }

    @Override
    public void simulationPeriodic() {
    }


}
