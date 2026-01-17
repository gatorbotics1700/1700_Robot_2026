package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.subsystems.mech.TransitionSubsystem;

public class TransitionCommand extends Command{

    private final TransitionSubsystem transitionSubsystem;

    private final double voltage;

    public TransitionCommand(TransitionSubsystem transitionSubsystem, double voltage){
        this.transitionSubsystem = transitionSubsystem;
        this.voltage = voltage;
        addRequirements(transitionSubsystem);
    }

    @Override
    public void execute(){
        transitionSubsystem.setMotorVoltage(voltage);
    }

    @Override
    public boolean isFinished(){
        if(voltage == 0){
            return true;
        }
        return false;
    }

}
