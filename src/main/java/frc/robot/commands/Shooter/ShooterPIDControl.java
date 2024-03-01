package frc.robot.commands.Shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterPIDControl extends Command
{
    private double degree;
    private ShooterSubsystem shooterSubsystem;
    private PIDController shooterPID;

    public ShooterPIDControl(double degree)
    {
        this.degree=degree;
        shooterSubsystem=ShooterSubsystem.getInstance();
        addRequirements(shooterSubsystem);
        shooterPID=new PIDController(2.5,0,0);
    }

    @Override
    public void initialize()
    {
        shooterPID.reset();
        shooterPID.setSetpoint(degree);
        shooterPID.setTolerance(0.01);
    }

    @Override
    public void execute()
    {
        shooterSubsystem.setPivotMotor(shooterPID.calculate(shooterSubsystem.getAbsoluteDegree()));        
    }

    @Override
    public void end(boolean interrupted)
    {
        shooterSubsystem.setPivotMotor(0);
    }

    @Override
    public boolean isFinished()
    {
        return shooterPID.atSetpoint();
    }
}
