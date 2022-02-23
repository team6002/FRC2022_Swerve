// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class SUB_Climber extends SubsystemBase {
  // Encoder ThroughBore;
  // private final Solenoid m_IntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
  /** Creates a new SUB_Climber. */
  public SUB_Climber() {
    // ThroughBore = new Encoder(2,3);
  }

  @Override
  public void periodic() {
    // int Ticks = ThroughBore.get(); // 2000 ticks are about a rotation.
    // SmartDashboard.putNumber("ThroughBore", Ticks);
    // // This method will be called once per schedule;r run
  }
}
