package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstant;

public class ClimberSubsystem extends SubsystemBase
{
    CANSparkMax Climber1;
    CANSparkMax Climber2;

    static ClimberSubsystem instance;

    public ClimberSubsystem()
    {
        Climber1 = new CANSparkMax(ClimberConstant.CLIMBER1_PORT, MotorType.kBrushless);
        Climber2 = new CANSparkMax(ClimberConstant.CLIMBER2_PORT, MotorType.kBrushless);

        Climber1.restoreFactoryDefaults();
        Climber2.restoreFactoryDefaults();

        Climber1.setIdleMode(IdleMode.kBrake);
        Climber2.setIdleMode(IdleMode.kBrake);
    }

    public void climberMotors(double speed)
    {
        Climber1.set(speed);
        Climber2.set(speed);
    }

    public static ClimberSubsystem getInstance()
    {
        if(instance==null)
        {
            instance=new ClimberSubsystem();
        }
        return instance;
    }
}
