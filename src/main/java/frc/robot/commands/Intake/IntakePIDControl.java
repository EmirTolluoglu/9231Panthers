package frc.robot.commands.Intake;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakePIDControl extends Command 
{
    private IntakeSubsystem intakeSubsystem;
    private double setPoint;
    private PIDController intakePID;

    public IntakePIDControl(double setPoint)
    {
        this.setPoint=setPoint;
        intakeSubsystem=IntakeSubsystem.getInstance();
        intakePID = new PIDController(7, 0.00, 0 );
        addRequirements(intakeSubsystem);
        SmartDashboard.putNumber("SetPoint", setPoint);
    }
    
    @Override
    public void initialize()
    {
        intakePID.reset();
        intakePID.setSetpoint(setPoint);
        intakePID.setTolerance(0.001);
    }
    
    @Override 
    public void execute()
    {
        intakeSubsystem.setPivotMotor(intakePID.calculate(intakeSubsystem.getAbsoluteEncoder()));
        SmartDashboard.putNumber("SetPoint", setPoint);
    }
    
    @Override 
    public void end(boolean interrupted)
    {
        intakeSubsystem.setPivotMotor(0);
    }

    @Override
    public boolean isFinished()
    {
        return intakePID.atSetpoint();
    }
}
