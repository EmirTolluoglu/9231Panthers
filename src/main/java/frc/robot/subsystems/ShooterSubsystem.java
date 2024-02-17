package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstant;

public class ShooterSubsystem extends SubsystemBase {
    
    CANSparkMax PivotMotor1 = new CANSparkMax(ShooterConstant.PIVOT_MOTOR1_PORT, MotorType.kBrushless);
    CANSparkMax PivotMotor2 = new CANSparkMax(ShooterConstant.PIVOT_MOTOR2_PORT, MotorType.kBrushless);
    
    CANSparkMax RollerMotor1 = new CANSparkMax(ShooterConstant.ROLLER_MOTOR1_PORT, MotorType.kBrushless);
    CANSparkMax RollerMotor2 = new CANSparkMax(ShooterConstant.ROLLER_MOTOR2_PORT, MotorType.kBrushless);

    static ShooterSubsystem instance;

    private RelativeEncoder pivot1Encoder;    
    private RelativeEncoder pivot2Encoder;

    double limit_max, limit_min;
    
    public ShooterSubsystem() {

        PivotMotor1.restoreFactoryDefaults();
        PivotMotor2.restoreFactoryDefaults();
        RollerMotor1.restoreFactoryDefaults();        
        RollerMotor2.restoreFactoryDefaults();
        
        PivotMotor1.setIdleMode(IdleMode.kBrake);        
        PivotMotor2.setIdleMode(IdleMode.kBrake);
        RollerMotor1.setIdleMode(IdleMode.kBrake);
        RollerMotor2.setIdleMode(IdleMode.kBrake);
        
        RollerMotor2.setInverted(true);
        PivotMotor2.setInverted(true);

        pivot1Encoder = PivotMotor1.getEncoder();
        pivot2Encoder = PivotMotor2.getEncoder();

    }

    public void setRollerMotor(double forward) {
        SmartDashboard.putNumber("Shooter Potencia (%)", forward * 100.0);
        RollerMotor1.set(forward);
        RollerMotor2.set(forward);
    }

    public void setPivotMotor(double degree) {
        SmartDashboard.putNumber("Shooter degree", degree);
        PivotMotor1.set(degree);
        PivotMotor2.set(degree);
    }

    public static ShooterSubsystem getInstance()
    {
        if (instance == null) 
        {
            instance = new ShooterSubsystem();
        }
        return instance;
    }

}
