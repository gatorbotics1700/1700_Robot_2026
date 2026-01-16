package frc.robot.subsystems.mech;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    public final TalonFX flywheelMotor;
    public final TalonFX hoodMotor;
    private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    public ShooterSubsystem(){
        flywheelMotor = new TalonFX(Constants.FLYWHEEL_MOTOR_CAN_ID);
        hoodMotor = new TalonFX(Constants.HOOD_MOTOR_CAN_ID);
    }

    public void setFlywheelVoltage(double voltage){
        flywheelMotor.setVoltage(voltage);
    }

    public void setHoodSpeed(double speed){
        hoodMotor.setControl(dutyCycleOut.withOutput(speed));
    }

    public double getPosition(){
        return hoodMotor.getPosition().getValueAsDouble() * 360 / Constants.HOOD_GEAR_RATIO; //TODO: check this conversion into degrees
    }
}
