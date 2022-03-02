// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

public class FSM_IntakeStatus {

    public enum IntakeState 
    {
        HOME
        ,INTAKE
        ,REVERSE
        ,SHOOTING
    }

    private IntakeState m_currentState = IntakeState.HOME;

    public void setState(IntakeState p_State) {
        m_currentState = p_State;
    }
    
    public IntakeState getState() {
        return m_currentState;
    }
    
    public boolean getState(IntakeState p_State) {
        return (m_currentState == p_State);
    }

    public boolean getState(String p_State) {
        return (m_currentState.toString().equals(p_State));
    }

    
}