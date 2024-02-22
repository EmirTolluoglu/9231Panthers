package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    
    final float kP = 0.3f, kI = 0.2f, kD = 0.3f; 
    CANSparkMax pivotMotor = new CANSparkMax(IntakeConstants.PIVOT_MOTOR_PORT, MotorType.kBrushless);
    CANSparkMax rollerMotor = new CANSparkMax(IntakeConstants.ROLLER_MOTOR_PORT, MotorType.kBrushless);
    PIDController pid = new PIDController(kP, kI, kD);
    private RelativeEncoder pivotEncoder;

    static IntakeSubsystem instance;

    double limit_max, limit_min;
    
    public IntakeSubsystem() 
    {
        pivotMotor.restoreFactoryDefaults();
        rollerMotor.restoreFactoryDefaults();

        pivotMotor.setIdleMode(IdleMode.kBrake);
        rollerMotor.setIdleMode(IdleMode.kBrake);

        pivotEncoder = pivotMotor.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature,IntakeConstants.kCPR);
    }

    public void setRollerMotor(double forward) {
        SmartDashboard.putNumber("Intake Potencia (%)", forward * 100.0);
        rollerMotor.set(forward);
    }

    public void setPivotMotor(double forward) {
        SmartDashboard.putNumber("Intake Degree (%)", forward * 100.0);
        pivotMotor.set(forward);
    }

    public double getPivotEncoder()
    {
        return pivotEncoder.getPosition();
    }

    public void resetPivotEncoder()
    {
        pivotEncoder.setPosition(0);
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Bore Encoder", getPivotEncoder());
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
