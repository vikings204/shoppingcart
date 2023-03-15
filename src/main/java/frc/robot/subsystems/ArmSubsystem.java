package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants204.Arm;
import frc.robot.Constants204.ArmCAN;

public class ArmSubsystem extends SubsystemBase {
    private final CANSparkMax boomMotor = new CANSparkMax(ArmCAN.BOOM_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder boomEncoder = boomMotor.getEncoder();
    private final SparkMaxPIDController boomPIDCon = boomMotor.getPIDController();
    private final CANSparkMax dipperMotor = new CANSparkMax(ArmCAN.DIPPER_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
    private final RelativeEncoder dipperEncoder = dipperMotor.getEncoder();
    private final SparkMaxPIDController dipperPIDCon = dipperMotor.getPIDController();
    private final Servo clawServo = new Servo(ArmCAN.CLAW_SERVO_PWM_CH);
    private boolean clawState = false;

    private final double kP = 0.5;
    private final double kI = 1e-4;
    private final double kD = 1;
    private final double kIz = 0;
    private final double kFF = 0;
    private final double kMaxOutput = 1;
    private final double kMinOutput = -1;

    public ArmSubsystem() {
        boomPIDCon.setP(kP);
        boomPIDCon.setI(kI);
        boomPIDCon.setD(kD);
        boomPIDCon.setIZone(kIz);
        boomPIDCon.setFF(kFF);
        boomPIDCon.setOutputRange(kMinOutput, kMaxOutput);
        dipperMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        dipperPIDCon.setP(kP);
        dipperPIDCon.setI(kI);
        dipperPIDCon.setD(kD);
        dipperPIDCon.setIZone(kIz);
        dipperPIDCon.setFF(kFF);
        dipperPIDCon.setOutputRange(kMinOutput, kMaxOutput);
        dipperMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    /**
     * boom increment, dipper increment, claw binary<br>
     * increment: 0=none, >0=increase, <0=decrease<br>
     * binary: 0=none, >0=opened, <0=closed<br>
    **/
    public void setArm(double b, double d, double c) {
        //System.out.println("B:"+b + "D:"+d + "C:"+c);
        double nb = boomEncoder.getPosition();
        double nd = dipperEncoder.getPosition();
        if (b == 0) {
        } else if (b > 0) {
            nb+=Arm.BOOM_REF_INCREMENT;
        } else if (b < 0) {
            nb-=Arm.BOOM_REF_INCREMENT;
        }
        if (d == 0) {
        } else if (d > 0) {
            nd+=Arm.DIPPER_REF_INCREMENT;
        } else if (d < 0) {
            nd-=Arm.DIPPER_REF_INCREMENT;
        }

        boomPIDCon.setReference(nb, CANSparkMax.ControlType.kPosition);
        dipperPIDCon.setReference(nd, CANSparkMax.ControlType.kPosition);
        if (c == 0) {
            if (clawState) {
                clawServo.set(Arm.CLAW_CLOSED_EXPOS);
            } else {
                clawServo.set(Arm.CLAW_OPEN_EXPOS);
            }
        } else if (c < 0) {
            clawState = true;
            clawServo.set(Arm.CLAW_CLOSED_EXPOS);
        } else if (c > 0) {
            clawState = false;
            clawServo.set(Arm.CLAW_OPEN_EXPOS);
        }
    }

    /*private double setServoScaleFix(double joyIn) {
        return (joyIn+1)/2;
    }*/
}