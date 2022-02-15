// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.subsystems.FSM_IntakeStatus;

public class TRG_Intake extends Button{
    private FSM_IntakeStatus m_IntakeStatus;
    private String m_wantedState;

    public TRG_Intake(FSM_IntakeStatus p_IntakeStatus, String p_wantedState)
    {
        m_IntakeStatus= p_IntakeStatus;
        m_wantedState = p_wantedState;
    }

    @Override
    public boolean get()
    {
        return m_IntakeStatus.getState(m_wantedState);
    }
}