package frc.robot.subsystems;

import java.io.Console;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase 
{    
    private static double kS = 0.00;
    private static double kG = 0;
    private static double kV = 0.0; 

    CANSparkMax pivotMotor;
    CANSparkMax rollerMotor;
    
    SparkPIDController pivotController;

    private RelativeEncoder boreEncoder;

    private DigitalInput limitSwitch=new DigitalInput(1);

    private ArmFeedforward pivotFeedForward;

    static IntakeSubsystem instance;

    static double setPoint;

    public IntakeSubsystem() 
    {
        pivotMotor = new CANSparkMax(IntakeConstants.PIVOT_MOTOR_PORT, MotorType.kBrushless);
        rollerMotor = new CANSparkMax(IntakeConstants.ROLLER_MOTOR_PORT, MotorType.kBrushless);
        
        pivotController= pivotMotor.getPIDController();

        pivotFeedForward = new ArmFeedforward(kS, kG, kV);

        boreEncoder = pivotMotor.getAlternateEncoder(Type.kQuadrature,8192);

        motorConfig();
    }

    public void motorConfig()
    {
        pivotMotor.restoreFactoryDefaults();
        rollerMotor.restoreFactoryDefaults();

        pivotMotor.setIdleMode(IdleMode.kCoast);
        rollerMotor.setIdleMode(IdleMode.kCoast);

        pivotMotor.setInverted(true);
    }

    public void setRollerMotor(double forward) {
        
        rollerMotor.set(forward);
    }

    public void setPivotMotor(double forward) {
        
        pivotMotor.set(forward);
    }

    public boolean getLimitSwitch()
    {
        return limitSwitch.get();
    }

    public void resetAbsoluteEncoder()
    {
    
        SmartDashboard.putBoolean("resetlendi", true);
    }

    public double getAbsoluteEncoder()
    {
        return boreEncoder.getPosition();
    }

    @Override
    public void periodic()
    {
        
        SmartDashboard.putNumber("Intake Bore", getAbsoluteEncoder());
        SmartDashboard.putBoolean("LimitW", getLimitSwitch());
        
        
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
