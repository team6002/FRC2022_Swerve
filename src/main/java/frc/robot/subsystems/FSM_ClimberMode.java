// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

public class FSM_ClimberMode {

    public enum ClimberState 
    {
        SHOOTING
        ,CLIMBING
    }

    private ClimberState m_currentState = ClimberState.SHOOTING;

    public void setState(ClimberState p_State) {
        m_currentState = p_State;
    }
    
    public ClimberState getState() {
        return m_currentState;
    }
    
    public boolean getState(ClimberState p_State) {
        return (m_currentState == p_State);
    }

    public boolean getState(String p_State) {
        return (m_currentState.toString().equals(p_State));
    }

    
}