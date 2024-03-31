// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeSubsystem extends SubsystemBase {

  private final TalonFX IntakeMotor;
  private final CANSparkMax secondaryIntake;
  private SparkPIDController SIPID;
  private RelativeEncoder enc;
  public double kP, kI, kD, kIz, kFF, kMax, kMin, maxRPM;
  private final DigitalInput RangeFinder;

  /** Creates a new Intake. */
  public IntakeSubsystem() {

IntakeMotor = new TalonFX(Constants.Intake.IntakeMotorID);
IntakeMotor.setInverted(true);

secondaryIntake = new CANSparkMax(20, CANSparkLowLevel.MotorType.kBrushless);
secondaryIntake.setInverted(true);
SIPID = secondaryIntake.getPIDController();
enc = secondaryIntake.getEncoder();
kP = 6e-5;
kI = 0;
kD = 0;
kIz = 0;
kFF = 0.000015;
kMax = 1;
kMin = -1;
maxRPM = 3000;
SIPID.setP(kP);
SIPID.setI(kI);
SIPID.setD(kD);
SIPID.setIZone(kIz);
SIPID.setFF(kFF);
SIPID.setOutputRange(kMin, kMax);
RangeFinder = new DigitalInput(3);

//RangeFinder.setAutomaticMode(true);




  }

  public void setspeed(Double speed){
IntakeMotor.set(speed);
secondaryIntake.set(speed * 0.75);
//SIPID.setReference(maxRPM, ControlType.kSmartVelocity);

  }

// public void IntakeFWD(){
//   IntakeMotor.set(ControlMode.PercentOutput, Constants.Intake.FWDSpeed);
// }

// public void IntakeREV(){
//   IntakeMotor.set(ControlMode.PercentOutput, Constants.Intake.REVSpeed);
// }

public void Stop(){

  IntakeMotor.set(0);
  secondaryIntake.set(0);
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run


SmartDashboard.putBoolean("Intake Beam", !RangeFinder.get());

  }



public boolean Note_In_Intake(){

  return !RangeFinder.get();

  




}





}
