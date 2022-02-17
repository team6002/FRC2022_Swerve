// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

public class FSM_IntakeStatus {

    public enum State 
    {
        HOME
        ,INTAKE
        ,REVERSE
        ,SHOOTING
    }

    private State m_currentState = State.HOME;

    public void setState(State p_State) {
        m_currentState = p_State;
    }
    
    public State getState() {
        return m_currentState;
    }
    
    public boolean getState(State p_State) {
        return (m_currentState == p_State);
    }

    public boolean getState(String p_State) {
        return (m_currentState.toString().equals(p_State));
    }

    
}