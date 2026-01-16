package frc.robot.subsystems.mech;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase{
    
    public final TalonFX motor;

    public IntakeSubsystem(){
        motor = new TalonFX(Constants.INTAKE_MOTOR_CAN_ID);
    }

    public void setMotorVoltage(double voltage){
        motor.setVoltage(voltage);
    }

    @Override
    public void periodic(){
    }

}
