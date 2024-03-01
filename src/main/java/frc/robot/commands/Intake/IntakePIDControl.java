package frc.robot.commands.Intake;

import edu.wpi.first.math.controller.PIDController;
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
        intakePID = new PIDController(7, 0, 0.06);
        addRequirements(intakeSubsystem);
    }
    
    @Override
    public void initialize()
    {
        intakePID.reset();
        intakePID.setSetpoint(setPoint);
        intakePID.setTolerance(0.01);
    }
    
    @Override 
    public void execute()
    {
        intakeSubsystem.setPivotMotor(intakePID.calculate(intakeSubsystem.getAbsoluteEncoder()));
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
