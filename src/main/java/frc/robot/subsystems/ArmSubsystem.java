// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.InvertType;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
//import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ArmSubsystem extends SubsystemBase {

  public final TalonFX PivotMotor;
  
  //private final DutyCycleEncoder HexEncoder;

  //private final Duty Hexencoder2;

 



 private DutyCycleOut m_DutyCycle = new DutyCycleOut(0);
  
private final CurrentLimitsConfigs m_currentLimits = new CurrentLimitsConfigs();

//public final DigitalInput m_FwdLimit = new DigitalInput(2);
//public final DigitalInput m_RevLimit = new DigitalInput(1);  ///////////////////////////////////////////////////////////////////////////



  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {

    PivotMotor = new TalonFX(Constants.Arm.PivotID);
    PivotMotor.setInverted(false);
    
   PivotMotor.setNeutralMode(NeutralModeValue.Brake);///
    //HexEncoder = new DutyCycleEncoder(Constants.Arm.EncoderPWMID);
    

   
    

    ConfigPivot();

     

  }

  public void ConfigPivot(){

TalonFXConfiguration toConfigure = new TalonFXConfiguration();

  
      //PivotMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,Constants.Arm.MAX_CURRENT_DRAW,Constants.Arm.MAX_CURRENT_DRAW + 5, 0.5));

    m_currentLimits.SupplyCurrentLimit = Constants.Arm.MAX_CURRENT_DRAW ; // Limit to 1 amps
    m_currentLimits.SupplyCurrentThreshold = Constants.Arm.MAX_CURRENT_DRAW + 5; // If we exceed 4 amps
    m_currentLimits.SupplyTimeThreshold = .5; // For at least 1 second
    m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it

    m_currentLimits.StatorCurrentLimit = 25; // Limit stator to 20 amps
    m_currentLimits.StatorCurrentLimitEnable = true; // And enable it

    toConfigure.CurrentLimits = m_currentLimits;

    toConfigure.SoftwareLimitSwitch.withForwardSoftLimitThreshold(20);
    toConfigure.SoftwareLimitSwitch.withReverseSoftLimitThreshold(0);
    toConfigure.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
    toConfigure.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
toConfigure.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    toConfigure.Slot0.kP = 3;
    toConfigure.Slot0.kI = 1;






    toConfigure.MotorOutput.NeutralMode = NeutralModeValue.Brake;  
    PivotMotor.getConfigurator().apply(toConfigure);
  }



      

  

  public void setspeed(Double speed){
PivotMotor.setControl(m_DutyCycle.withOutput(speed)
//.withLimitForwardMotion(!m_FwdLimit.get())
//.withLimitReverseMotion(!m_RevLimit.get())
);


  }


//   public void LimitresetLow(){
// //if (!m_RevLimit.get()) {

//   if (PivotMotor.getReverseLimit().getValueAsDouble() > 1){

//   ZeroPivotPositon();

// }


// }


  public void stop(){
    PivotMotor.set( 0);
  
    
      }

      public double getPivotEncoder(){

       return PivotMotor.getPosition().getValueAsDouble();

      } 




//     public double Calibrate_Arm_Encoder_To_Hex_Encoder(){

//       double inputangle = HexEncoder.getAbsolutePosition();
      
// if (inputangle > .25 && encoderupdated == false) {


  
//   Timer.delay(3);
//   inputangle = HexEncoder.getAbsolutePosition();

//   encoderupdated = true; 

// }
//      // double CalculatedEnocderPosition = (1287.7292741*(Math.pow(inputangle,2))+(-2760.633858*inputangle)+(1338.221256));
//  double CalculatedEnocderPosition = ((-48914*(Math.pow(inputangle, 4))) + (140171 * ( Math.pow(inputangle, 3))) + (-146621*(Math.pow(inputangle, 2))) + (65466*inputangle) + (-10284));
//       return CalculatedEnocderPosition;

//PivotMotor.setPosition(CalculatedEnocderPosition);

    //}


//     public void setpivotpoistion(){

     
      
// if (encoderupdated == false) {

//       PivotMotor.setPosition(Calibrate_Arm_Encoder_To_Hex_Encoder());
    
//   }}

  public void ZeroPivotPositon(){

    PivotMotor.setPosition(0);
  }

  @Override
  public void periodic() {

   // LimitresetLow();


    //SmartDashboard.putNumber("Hex Encoder Value", HexEncoder.getAbsolutePosition());
    SmartDashboard.putNumber("Pivot Motor Encoder", getPivotEncoder());
    SmartDashboard.putNumber("Pivot Motor Current",PivotMotor.getStatorCurrent().getValueAsDouble());
    //SmartDashboard.putNumber("Calc Encoder Position", Calibrate_Arm_Encoder_To_Hex_Encoder());

    SmartDashboard.putBoolean("FWD Limit Arm", PivotMotor.getForwardLimit().getValue()== ForwardLimitValue.ClosedToGround);
    SmartDashboard.putBoolean("Rev Limit Arm", PivotMotor.getReverseLimit().getValue() == ReverseLimitValue.ClosedToGround);


    // This method will be called once per scheduler run
  }

// public void ConfigSoftlimits() {


//       //PivotMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true,Constants.Arm.MAX_CURRENT_DRAW,Constants.Arm.MAX_CURRENT_DRAW + 5, 0.5));
// TalonFXConfiguration toConfigure = new TalonFXConfiguration();
//     m_currentLimits.SupplyCurrentLimit = Constants.Arm.MAX_CURRENT_DRAW ; // Limit to 1 amps
//     m_currentLimits.SupplyCurrentThreshold = Constants.Arm.MAX_CURRENT_DRAW + 5; // If we exceed 4 amps
//     m_currentLimits.SupplyTimeThreshold = .5; // For at least 1 second
//     m_currentLimits.SupplyCurrentLimitEnable = true; // And enable it

//     m_currentLimits.StatorCurrentLimit = 25; // Limit stator to 20 amps
//     m_currentLimits.StatorCurrentLimitEnable = true; // And enable it

    
//     toConfigure.MotorOutput.NeutralMode = NeutralModeValue.Brake;
//     toConfigure.CurrentLimits = m_currentLimits;

//     toConfigure.SoftwareLimitSwitch.withForwardSoftLimitThreshold(0);
//     toConfigure.SoftwareLimitSwitch.withReverseSoftLimitThreshold(-20);
//     toConfigure.SoftwareLimitSwitch.withForwardSoftLimitEnable(true);
//     toConfigure.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
      


//     //toConfigure.MotorOutput.withMotorOutput(NeutralMode.Brake);
    

//     PivotMotor.getConfigurator().apply(toConfigure);

// }



}
