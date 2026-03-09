package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;

public class Intake {
    
    public static TalonFX intakeMotor1 = new TalonFX(10);
    public static TalonFX intakeMotor2 = new TalonFX(11);

       public static void intakeFoward(){
        intakeMotor1.set(1);
        intakeMotor2.set(1);
    }

    public static void intakeReverse(){
        intakeMotor1.set(-1);
        intakeMotor2.set(-1);
    }

}
