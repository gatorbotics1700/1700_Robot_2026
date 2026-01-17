package frc.robot.subsystems.mech;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TransitionSubsystem extends SubsystemBase {
    public final TalonFX kickerMotor;
    public final TalonFX hopperMotor;


    public TransitionSubsystem(){
        kickerMotor = new TalonFX(Constants.KICKER_MOTOR_CAN_ID);
        hopperMotor = new TalonFX(Constants.HOPPER_MOTOR_CAN_ID);
    }

    public void periodic(){

    }

    public void setVoltages(double kickerVoltage, double hopperVoltage){
        kickerMotor.setVoltage(kickerVoltage);
        hopperMotor.setVoltage(hopperVoltage);
    }

}
