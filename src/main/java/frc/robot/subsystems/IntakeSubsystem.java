package frc.robot.subsystems;

import java.io.Console;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    
     
    CANSparkMax pivotMotor = new CANSparkMax(IntakeConstants.PIVOT_MOTOR_PORT, MotorType.kBrushless);
    CANSparkMax rollerMotor = new CANSparkMax(IntakeConstants.ROLLER_MOTOR_PORT, MotorType.kBrushless);
    
    private DutyCycleEncoder absoluteEncoder;

    static IntakeSubsystem instance;


    
    public IntakeSubsystem() 
    {
        pivotMotor.restoreFactoryDefaults();
        rollerMotor.restoreFactoryDefaults();

        pivotMotor.setIdleMode(IdleMode.kBrake);
        rollerMotor.setIdleMode(IdleMode.kBrake);

        absoluteEncoder= new DutyCycleEncoder(0);
        
    }

    public void setRollerMotor(double forward) {
        
        rollerMotor.set(forward);
    }

    public void setPivotMotor(double forward) {
        
        pivotMotor.set(forward);
    }

    

    public void resetAbsoluteEncoder()
    {
        absoluteEncoder.reset();
        SmartDashboard.putBoolean("resetlendi", true);
    }

    public double getAbsoluteEncoder()
    {
        return ((absoluteEncoder.get()<0.5f)? absoluteEncoder.get()+1 : absoluteEncoder.get());
    }

    @Override
    public void periodic()
    {
        
        SmartDashboard.putNumber("Daha da Bore", getAbsoluteEncoder());
        
    }

    public static IntakeSubsystem getInstance()
    {
        if(instance==null)
        {
            instance=new IntakeSubsystem();
        }
        return instance;
    }
}
