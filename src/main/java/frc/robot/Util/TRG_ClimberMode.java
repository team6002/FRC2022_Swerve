// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.subsystems.FSM_ClimberMode;
import frc.robot.subsystems.FSM_ClimberMode.ClimberState;

public class TRG_ClimberMode extends Button{
    private FSM_ClimberMode m_climberMode;
    private ClimberState m_wantedState;

    public TRG_ClimberMode(FSM_ClimberMode p_climberMode, ClimberState p_wantedState)
    {
        m_climberMode= p_climberMode;
        m_wantedState = p_wantedState;
    }

    @Override
    public boolean get()
    {
        return m_climberMode.getState(m_wantedState);
    }
}