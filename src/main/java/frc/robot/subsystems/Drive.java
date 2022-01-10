/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Drive extends SubsystemBase {
  
  // Declare drive motor controllers
   private CANSparkMax leftFront, leftRear, rightFront, rightRear;
   private CANCoder leftEncoder, rightEncoder;
   private SparkMaxPIDController leftPID, rightPID;
   

  // Creates a new drive
  public Drive() {

    // Initialize motor controllers using ID's stored in Constants
    leftFront = new CANSparkMax(Constants.ID_LEFT_FRONT, MotorType.kBrushless);
    rightFront = new CANSparkMax(Constants.ID_RIGHT_FRONT, MotorType.kBrushless);
    leftRear = new CANSparkMax(Constants.ID_LEFT_REAR, MotorType.kBrushless);
    rightRear = new CANSparkMax(Constants.ID_RIGHT_REAR, MotorType.kBrushless);

    // Set right front controller to be inverted
    leftFront.setInverted(false);
    rightFront.setInverted(true);

    // Set rear controllers to follow front
    leftRear.follow(leftFront, false);
    rightRear.follow(rightFront, false);

    leftPID = leftFront.getPIDController();
    rightPID = rightFront.getPIDController();

    leftEncoder = leftFront.getEncoder(encoderType, countsPerRev)
    rightEncoder = rightFront.getEncoder();
  }

  //Resets neo drive motor encoders
  public void resetEncoders(){
    rightEncoder.setPosition(0);
    leftEncoder.setPosition(0);
  }

  public void setkP(double kP){
    leftPID.setP(kP);
    rightPID.setP(kP);
  }

  public void setkI(double kI){
    leftPID.setI(kI);
    rightPID.setI(kI);
  }

  public void setkD(double kD){
    leftPID.setD(kD);
    rightPID.setD(kD);
  }

  public void setkF(double kF){
    leftPID.setFF(kF);
    rightPID.setFF(kF);
  }
  // Methods to set speed of left and right motor controllers
  public void setLeftSpeed(double speed) {
    leftFront.set(speed);
  }

  public void setRightSpeed(double speed) {
    rightFront.set(speed);
  }

  // Set left and right drive speeds to zero
  public void stop(){
    setLeftSpeed(0);
    setRightSpeed(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putNumber("Left", getLeftEncoder());
    SmartDashboard.putNumber("Right", getRightEncoder());


  }

public double getLeftEncoder() {
  return leftEncoder.getPosition() * Constants.DriveConstants.REV_TO_IN_K;
}
public double getRightEncoder() {
  return rightEncoder.getPosition() * Constants.DriveConstants.REV_TO_IN_K;
}
/*
public double getRightDistance() {
  return getRightEncoder(); 
}
*/
public void setOutputRange(double minOutput, double maxOutput) {
  leftPID.setOutputRange(minOutput, maxOutput);
  rightPID.setOutputRange(minOutput, maxOutput);
}
  public void setSetPoint(double dist) {
    leftPID.setReference((dist * Constants.DriveConstants.IN_TO_REV_K), ControlType.kPosition);
    rightPID.setReference((dist * Constants.DriveConstants.IN_TO_REV_K), ControlType.kPosition);
  }

  
}
