package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mech.ClimberSubsystem;
import frc.robot.subsystems.mech.IntakeSubsystem;
import frc.robot.Constants;

public class ClimberCommand extends Command {
    private final ClimberSubsystem climberSubsystem;
    private final IntakeSubsystem intakeSubsystem;
    private int level;

    public ClimberCommand(ClimberSubsystem climberSubsystem, IntakeSubsystem intakeSubsystem, int level){
        this.climberSubsystem = climberSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.level = level;
        
        addRequirements(climberSubsystem, intakeSubsystem);
    }

    @Override
    public void execute(){
        intakeSubsystem.extendIntake(false);
        if (level == 0){
            double groundPosition = climberSubsystem.getCurrentTicksOuter() - climberSubsystem.InchesToTicks(Constants.LOW_RUNG_ARM_LENGTH);
            climberSubsystem.extendOuterArm(groundPosition);
        } else if(level == 1){
            double LowPosition = climberSubsystem.getCurrentTicksOuter() + climberSubsystem.InchesToTicks(Constants.LOW_RUNG_ARM_LENGTH);
            climberSubsystem.extendOuterArm(LowPosition);
            double lowRetracted = climberSubsystem.getCurrentTicksOuter() + climberSubsystem.InchesToTicks(Constants.LOW_RUNG_ARM_RETRACTED);
            climberSubsystem.extendOuterArm(lowRetracted);
        } else if (level == 2){

        } else if (level == 3){
            double lowPosition = climberSubsystem.getCurrentTicksOuter() + climberSubsystem.InchesToTicks(Constants.LOW_RUNG_ARM_LENGTH);
            climberSubsystem.extendOuterArm(lowPosition);
            double lowRetracted = climberSubsystem.getCurrentTicksOuter() + climberSubsystem.InchesToTicks(Constants.LOW_RUNG_ARM_RETRACTED);
            climberSubsystem.extendOuterArm(lowRetracted);
            double midPosition = climberSubsystem.getCurrentTicksOuter() + climberSubsystem.InchesToTicks(Constants.MID_RUNG_ARM_LENGTH);
            climberSubsystem.extendInnerArm(midPosition);
            midPosition = climberSubsystem.getCurrentTicksInner() + climberSubsystem.InchesToTicks(Constants.MID_RUNG_ARM_LENGTH);  //check if arms will have to extend different lengths here
            climberSubsystem.extendOuterArm(midPosition);
        } else {
            System.out.println("*** NOT VALID LEVEL ***");
        }
    }

}
