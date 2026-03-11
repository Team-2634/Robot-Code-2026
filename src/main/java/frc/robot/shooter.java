

package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;


public class shooter {

    public static TalonFX shooterMotor1 = new TalonFX(10);
    public static TalonFX shooterMotor2 = new TalonFX(11);


    public static void shooterFoward(){
        shooterMotor1.set(1);
        shooterMotor2.set(1);
    }

    public static void shooterStop(){
        shooterMotor1.set(0);
        shooterMotor2.set(0);
    }

}
