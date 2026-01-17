package frc.robot.subsystems.mech;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    public final TalonFX flywheelMotor;
    public final TalonFX hoodMotor;
    private boolean isTargetting;
    private double hoodVoltage;
    private PIDController pidController;
    private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    public ShooterSubsystem(){
        flywheelMotor = new TalonFX(Constants.FLYWHEEL_MOTOR_CAN_ID);
        hoodMotor = new TalonFX(Constants.HOOD_MOTOR_CAN_ID);
        isTargetting = true;
    }

    public void periodic(){
        if(isTargetting){
            turnToPosition(getTargetPosition());
        }
    }

    public void setFlywheelVoltage(double flywheelVoltage){
        flywheelMotor.setVoltage(flywheelVoltage);
    }

    public void setHoodSpeed(double speed){
        hoodMotor.setControl(dutyCycleOut.withOutput(speed));
    }

    public double getPosition(){
        return hoodMotor.getPosition().getValueAsDouble() * 360 / Constants.HOOD_GEAR_RATIO; //TODO: check this conversion into degrees
    }

    public double getTargetPosition(){
        return 45; // replace with equation that calculates desired distance based off distance from shooter
    }

    public void turnToPosition(double targetPosition){
        hoodVoltage = pidController.calculate(targetPosition - getPosition());
        setHoodSpeed(hoodVoltage);
    }

    public void setIsTargetting(boolean isTargetting){
        this.isTargetting = isTargetting;
    }

}
