package frc.robot.subsystems.mech;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix6.controls.DutyCycleOut;
import frc.robot.Constants;



public class ClimberSubsystem extends SubsystemBase {
    
    public final TalonFX outerArmMotor;
    public final TalonFX innerArmMotor;
    private boolean isL3;

    private static DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
    private final PIDController pidController;

    private static final double kP = 0.0; //TODO: tune all of these
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double DEADBAND = 200;

    public ClimberSubsystem(){
        outerArmMotor = new TalonFX(Constants.OUTER_ARM_MOTOR_CAN_ID);
        innerArmMotor = new TalonFX(Constants.INNER_ARM_MOTOR_CAN_ID);
        isL3 = true;
        pidController = new PIDController(kP, kI, kD);
    }

    public void setOuterArmSpeed(double output){
        outerArmMotor.setControl(dutyCycleOut.withOutput(output));
    }

    public void setInnerArmSpeed(double output){
        innerArmMotor.setControl(dutyCycleOut.withOutput(output));
    }

    public void extendOuterArm(double desiredTicks){
        double currentTicks = getCurrentTicksOuter();
        double error = desiredTicks - currentTicks;
        if(Math.abs(error)> DEADBAND) {
            double output = pidController.calculate(error);
            setOuterArmSpeed(output);
        } else{
            setOuterArmSpeed(0);
        }
    }

    public void extendInnerArm(double desiredTicks){
        double currentTicks = getCurrentTicksInner();
        double error = desiredTicks - currentTicks;
        if(Math.abs(error)> DEADBAND){
            double output = pidController.calculate(error);
            setInnerArmSpeed(output);
        } else{
            setInnerArmSpeed(0);
        }
    }

    public void setLevel(boolean isL3){
        this.isL3=isL3;
    }

    public boolean getLevel(){
        return isL3;
    }

    public double getCurrentTicksOuter(){
        return outerArmMotor.getPosition().getValueAsDouble() * Constants.KRAKEN_TICKS_PER_REV;
    }

    public double getCurrentTicksInner(){
        return innerArmMotor.getPosition().getValueAsDouble() * Constants.KRAKEN_TICKS_PER_REV;
    }

    public double InchesToTicks(double desiredInches){
        return desiredInches * Constants.CLIMBER_TICKS_PER_INCH;
    }

}
