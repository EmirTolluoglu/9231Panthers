package frc.robot.subsystems;

import java.time.Period;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstant;
import frc.robot.LimelightHelpers;
public class ShooterSubsystem extends SubsystemBase {
    private PIDController shooterPID;
    CANSparkMax PivotMotor1 = new CANSparkMax(ShooterConstant.PIVOT_MOTOR1_PORT, MotorType.kBrushless);
    CANSparkMax PivotMotor2 = new CANSparkMax(ShooterConstant.PIVOT_MOTOR2_PORT, MotorType.kBrushless);
    
    CANSparkMax RollerMotor1 = new CANSparkMax(ShooterConstant.ROLLER_MOTOR1_PORT, MotorType.kBrushless);
    CANSparkMax RollerMotor2 = new CANSparkMax(ShooterConstant.ROLLER_MOTOR2_PORT, MotorType.kBrushless);

    static ShooterSubsystem instance;

    static DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(9);
    public double degreeAim;
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
        PivotMotor1.setInverted(true);

        absoluteEncoder.reset();

        shooterPID=new PIDController(3,0,0);
        PIDinitialize(1.05);
    }
    private void PIDinitialize(double degree)
    {
        degreeAim = degree;
        shooterPID.reset();
        shooterPID.setSetpoint(degree);
        shooterPID.setTolerance(0.01);
        
    }
    public void changeDegreeAim(double degree) {
        PIDinitialize(degree);
    }

    public void setRollerMotor(double forward) {
        
        RollerMotor1.setVoltage(forward);
        RollerMotor2.setVoltage(forward);
    }

    public void setPivotMotor(double degree) {
        
        PivotMotor1.set(degree);
        PivotMotor2.set(degree);
    }

    //get degree
    public double getAbsoluteDegree()
    {
        double bizim_encoder=(absoluteEncoder.getAbsolutePosition()<0.5f)? absoluteEncoder.getAbsolutePosition()+1:absoluteEncoder.getAbsolutePosition() ;
        return bizim_encoder;
    }

    @Override
    public void periodic()
    {
        SmartDashboard.putNumber("Shooter Bore",getAbsoluteDegree());
        SmartDashboard.putNumber("Setpoint", degreeAim);
        double newDegree = SmartDashboard.getNumber("SetPoint", 1.05);
        if(newDegree != degreeAim)
        {
            changeDegreeAim(newDegree);
        }
        setPivotMotor(shooterPID.calculate(getAbsoluteDegree()));

        if(LimelightHelpers.getTV("limelight"))
        {
            SmartDashboard.putNumber("LIMLIT", LimelightHelpers.getTY("limelight"));
            changeDegreeAim(1-((LimelightHelpers.getTY("limelight")/400)));
        }
    }

    public static ShooterSubsystem getInstance()
    {
        if (instance == null) 
        {
            instance = new ShooterSubsystem();
        }
        return instance;
    }

    public static final InterpolatingDoubleTreeMap DISTANCE_TO_ANGLE_MAP = new InterpolatingDoubleTreeMap();

    static {
        //DISTANCE_TO_ANGLE_MAP.put(1.25, ArmConstants.kSUBWOOFER);
        //DISTANCE_TO_ANGLE_MAP.put(2.2, ArmConstants.kOffset - 0.075);
        //DISTANCE_TO_ANGLE_MAP.put(3.0, ArmConstants.kOffset - 0.058);
        //DISTANCE_TO_ANGLE_MAP.put(4.1, ArmConstants.kOffset - 0.038);
        //DISTANCE_TO_ANGLE_MAP.put(4.9, ArmConstants.kOffset - 0.035);
        //DISTANCE_TO_ANGLE_MAP.put(5.5, ArmConstants.kOffset - 0.028);
      }
  
}
