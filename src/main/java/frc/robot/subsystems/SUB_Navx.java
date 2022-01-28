// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class SUB_Navx extends SubsystemBase {
  /** Creates a new SUB_Navx. */
  private AHRS NavxGyro = new AHRS(Port.kMXP);
  public SUB_Navx() {
    NavxGyro.reset();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("NavxGyroAngle",NavxGyro.getAngle());
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("NavxDegrees", NavxGyro.getRotation2d().getDegrees());
  }

  public double getAngle(){
      return NavxGyro.getAngle();
  }
  
  public Rotation2d getRotation2d(){
    return NavxGyro.getRotation2d();
  }
  public double getDegrees(){
    return NavxGyro.getRotation2d().getDegrees();
  }

  public void resetNavx(){
    NavxGyro.reset();
  }


}
