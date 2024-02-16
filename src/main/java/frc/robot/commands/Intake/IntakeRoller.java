package frc.robot.commands.Intake;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeRoller extends Command 
{
    private IntakeSubsystem intakeSubsystem;
    private double speed;

    public IntakeRoller(double speed)
    {
        intakeSubsystem=IntakeSubsystem.getInstance();
        this.speed=speed;
    }

    @Override
    public void execute()
    {
        intakeSubsystem.setPivotMotor(speed);
    }

    @Override
    public void end(boolean interrupted)
    {
        intakeSubsystem.setPivotMotor(0);
    }
}