// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ArmPivot;


import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.proto.Wpimath;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class PIDPivotCommand extends Command {
  /** Creates a new PIDPivotCommand. */
  
  private final ArmSubsystem m_ArmSubsystem;
  private double setPoint;
  private PositionVoltage m_armPositionVoltage;

  private String PiovtSETpointSD = "Pivot Target";

  
  public PIDPivotCommand(ArmSubsystem m_ArmSubsystem) {
    this.m_ArmSubsystem = m_ArmSubsystem;
    
  
   // m_PivotPIDController.enableContinuousInput(-1, 1);
       m_armPositionVoltage = new PositionVoltage(0);
    
    addRequirements(m_ArmSubsystem);

    

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

 SmartDashboard.setDefaultNumber(PiovtSETpointSD,0);

  

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {

    setPoint = SmartDashboard.getNumber(PiovtSETpointSD, 0);

    setPoint = MathUtil.clamp(setPoint, 0, 20);

    m_ArmSubsystem.PivotMotor.setControl(m_armPositionVoltage.withPosition(setPoint));



    SmartDashboard.putNumber("Runningvalue",setPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    //m_ArmSubsystem.setspeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
 
    return false;

  }


  



}
