package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.IntakeSubsystem;

public class IntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;

    private final double voltage;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, double voltage){
        this.intakeSubsystem = intakeSubsystem;
        this.voltage = voltage;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void execute(){
        intakeSubsystem.setMotorVoltage(voltage);
    }

    @Override
    public boolean isFinished(){
        if(voltage == 0){
            return true;
        }
        //potentially consider adding connection to vision, if we don't see any fuel
        return false;
    }

}
