package frc.robot.subsystems.mech;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TransitionSubsystem extends SubsystemBase {
    public final TalonFX transitionMotor;

    public TransitionSubsystem(){
        transitionMotor = new TalonFX(Constants.TRANSITION_MOTOR_CAN_ID);
    }

    public void periodic(){

    }

    public void setMotorVoltage(double voltage){
        transitionMotor.setVoltage(voltage);
    }
}
