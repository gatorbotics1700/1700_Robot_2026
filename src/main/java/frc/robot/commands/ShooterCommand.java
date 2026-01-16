package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.mech.ShooterSubsystem;

public class ShooterCommand extends Command{
    private ShooterSubsystem shooterSubsystem;
    private double anglePosition;
    private PIDController pidController;
    private double flywheelVoltage;

    public ShooterCommand(ShooterSubsystem shooterSubsystem, double anglePosition){
        this.shooterSubsystem = shooterSubsystem;
        this.anglePosition = anglePosition;
        addRequirements(shooterSubsystem);
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        shooterSubsystem.setFlywheelVoltage(flywheelVoltage);
        if(Math.abs(shooterSubsystem.getPosition() - anglePosition) > 2){
            double output = pidController.calculate(shooterSubsystem.getPosition() - anglePosition);
            shooterSubsystem.setHoodSpeed(output);
        } else{
            shooterSubsystem.setHoodSpeed(0);
        }
    }
}
