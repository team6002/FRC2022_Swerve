package frc.robot.Util;



import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.subsystems.FSM_HopperState;

public class TRG_Hopper extends Button{
    private String m_wantedState;
    private FSM_HopperState m_FSMHopperState;
    public TRG_Hopper(FSM_HopperState p_FSM_HopperState, String p_wantedState)
    {
        m_FSMHopperState = p_FSM_HopperState;
        m_wantedState = p_wantedState;
    }

    @Override
    public boolean get()
    {
        return m_FSMHopperState.getState(m_wantedState);
    }
}