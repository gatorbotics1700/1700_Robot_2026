package frc.robot.subsystems.mech;


import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

public class TalonLogger{
    private TalonFX talon;
    private String talonName;
    private String subsystem;

    public static Boolean DEBUG_LOGGING = true;

    private StatusSignal<?>[] statusSignalArray;



    public TalonLogger(String talonName, String subsystem, TalonFX talon){
        
        talonName = this.talonName;
        subsystem = this.subsystem;
        talon = this.talon;

        ArrayList<StatusSignal<?>> statusSignalList = new ArrayList<>();

        statusSignalList.add(talon.getMotorVoltage());
        statusSignalList.add(talon.getStatorCurrent());
        statusSignalList.add(talon.getStatorCurrent());
        statusSignalList.add(talon.getPosition());

        if(DEBUG_LOGGING){
            statusSignalList.add(talon.getClosedLoopReference());
            statusSignalList.add(talon.getClosedLoopError());
            statusSignalList.add(talon.getClosedLoopFeedForward());
        }

        statusSignalArray = statusSignalList.toArray(new StatusSignal<?>[0]);
    }

    private String createKey(String metric){
        String format = "%/%/%";
        return String.format(format, subsystem, talon, metric);
    }

    public void log(){
        BaseStatusSignal.refreshAll(statusSignalArray);
        for(StatusSignal<?> signal : statusSignalArray){
            Logger.recordOutput(createKey(signal.getName()), signal.getValueAsDouble());
        }
    }
}
