package frc.robot.subsystems;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.Intake.IntakePIDControl;
import frc.robot.commands.Intake.IntakePivot;
import frc.robot.commands.Intake.IntakeRoller;
import frc.robot.commands.Shooter.ShooterPIDControl;
import frc.robot.commands.Shooter.ShooterPivot;
import frc.robot.commands.Shooter.ShooterRoller;

public class DriverControlsSubsystem extends SubsystemBase{
    
    static DriverControlsSubsystem instance;
    private ClimberSubsystem m_climber;
    private IntakeSubsystem m_intake;
    private ShooterSubsystem m_shooter;
    private XboxController driverController;
    private PS4Controller operatorController;

    public DriverControlsSubsystem(){
        driverController = new XboxController(Constants.OperatorConstants.kDriverControllerPort);
        operatorController = new PS4Controller(Constants.OperatorConstants.kOperatorControllerPort);
        m_climber=ClimberSubsystem.getInstance();
        m_intake=IntakeSubsystem.getInstance();
    }

    public XboxController getDriverController(){
        return driverController;
    }

    public PS4Controller getOperatorController()
    {
        return operatorController;
    }

    /*Intake roller */
    public boolean IntakeRollerIn(){
        return driverController.getBButton();
    }
    
    public boolean IntakeRollerOut(){
        return driverController.getAButton();
    }

    public boolean IntakeRolllerZero()
    {
        return (driverController.getPOV() == 0)|| driverController.getRightStickButton();
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
    public boolean Climber1Positive(){
        return operatorController.getTriangleButton();
    }
    public boolean Climber1Negative(){
        return operatorController.getCircleButton();
    }

    public boolean Climber2Positive()
    {
        return operatorController.getSquareButton();
    }
    
    public boolean Climber2Negative()
    {
        return operatorController.getCrossButton();
    }

    public boolean IntakeMidPID()
    {
        return operatorController.getR1Button();
    }

    public boolean IntakeOutPID()
    {
        return operatorController.getL1Button();
    }

    public boolean shooterTopPID(){
        return driverController.getRightTriggerAxis()>0;
    }

    public boolean shooterDownPID()
    {
        return driverController.getPOV()==90;
    }

    public void setRumble(double speed){
        driverController.setRumble(RumbleType.kBothRumble, speed);
    }



    public void registerTriggers(){

    // Intake
   new Trigger(this::IntakeRollerOut).onTrue(new InstantCommand(()->m_intake.setRollerMotor(Constants.IntakeConstants.AMP_SHOOT_POWER)))
                                        .onFalse(new InstantCommand(()->m_intake.setRollerMotor(0)));
    new Trigger(this::IntakeRollerIn).onTrue(new IntakeRoller(-Constants.IntakeConstants.ROLLER_POWER))
                                        .onFalse(new IntakeRoller(0));
    


    // Intake
    new Trigger(this::IntakePivotPositive).onTrue(new InstantCommand(()->m_intake.setPivotMotor(Constants.IntakeConstants.PIVOT_POWER)))
                                            .onFalse(new InstantCommand(()->m_intake.setPivotMotor(0)));

    new Trigger(this::IntakePivotNegative).onTrue(new InstantCommand(()->m_intake.setPivotMotor(-Constants.IntakeConstants.PIVOT_POWER)))
                                            .onFalse(new InstantCommand(()->m_intake.setPivotMotor(0)));

    // Shooter
    new Trigger(this::ShooterPivotPositive).onTrue(new ShooterPivot(Constants.ShooterConstant.PIVOT_POWER))
                                            .onFalse(new ShooterPivot(0));
    new Trigger(this::ShooterPivotNegative).onTrue(new ShooterPivot(-Constants.ShooterConstant.PIVOT_POWER))
                                            .onFalse(new ShooterPivot(0));

    // Shooter
    new Trigger(this::ShooterRoller).onTrue(new ShooterRoller(Constants.ShooterConstant.ROLLER_POWER))
                                        .onFalse(new ShooterRoller(0));

    // Climber
    new Trigger(this::Climber1Positive).onTrue(new InstantCommand(()->m_climber.climber1Motor(Constants.ClimberConstant.CLIMBER_POWER)))
                                        .onFalse(new InstantCommand(()->m_climber.climber1Motor(0)));
    new Trigger(this::Climber1Negative).onTrue(new InstantCommand(()->m_climber.climber1Motor(-Constants.ClimberConstant.CLIMBER_POWER)))
                                        .onFalse(new InstantCommand(()->m_climber.climber1Motor(0)));
    new Trigger(this::Climber2Positive).onTrue(new InstantCommand(()->m_climber.climber2Motor(Constants.ClimberConstant.CLIMBER_POWER)))
                                        .onFalse(new InstantCommand(()->m_climber.climber2Motor(0)));
    new Trigger(this::Climber2Negative).onTrue(new InstantCommand(()->m_climber.climber2Motor(-Constants.ClimberConstant.CLIMBER_POWER)))
                                        .onFalse(new InstantCommand(()->m_climber.climber2Motor(0)));

    //PID
    new Trigger(this::IntakeMidPID).onTrue(new IntakePIDControl(1.04));
    new Trigger(this::IntakeOutPID).onTrue(new IntakePIDControl(1.3));
    new Trigger(this::shooterTopPID).onTrue(new ShooterPIDControl(0.95));
    new Trigger(this::shooterDownPID).onTrue(new ShooterPIDControl(1.05));
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
            //setRumble(1);
        }
        else if(DriverStation.getMatchTime() < 10){
            //setRumble(1);
        }
    }

    @Override
    public void simulationPeriodic() {
    }


}
