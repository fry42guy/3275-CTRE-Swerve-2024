// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeFWDWithSensor extends Command {
  private final IntakeSubsystem m_IntakeSubsystem;
  private final XboxController controller;
/** Creates a new IntakeFWD. */
  public IntakeFWDWithSensor(IntakeSubsystem m_IntakeSubsystem, XboxController controller) {

    this.m_IntakeSubsystem = m_IntakeSubsystem;
    this.controller = controller;

    addRequirements(m_IntakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

if (m_IntakeSubsystem.Note_In_Intake()){
          // if (m_ShooterSubsystem.getisActive() == false){
  m_IntakeSubsystem.Stop();
  controller.setRumble(GenericHID.RumbleType.kBothRumble, 0.5);
}
    else {
m_IntakeSubsystem.setspeed(Constants.Intake.FWDSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    m_IntakeSubsystem.Stop();
      controller.setRumble(GenericHID.RumbleType.kBothRumble, 0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
