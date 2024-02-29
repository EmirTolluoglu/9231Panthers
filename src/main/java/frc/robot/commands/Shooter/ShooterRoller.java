package frc.robot.commands.Shooter;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ShooterRoller extends Command
{
    ShooterSubsystem shooterSubsystem ;

    double speed;
    public ShooterRoller(double speed)
    {
        this.speed=speed;
        shooterSubsystem= ShooterSubsystem.getInstance();
        addRequirements(shooterSubsystem);
    }
    
    @Override
    public void initialize()
    {
        shooterSubsystem.setRollerMotor(speed);
    }
    
    
    @Override 
    public void end(boolean interrupted)
    {
        shooterSubsystem.setRollerMotor(0);
        
    }
}
