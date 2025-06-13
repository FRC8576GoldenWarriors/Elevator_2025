// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private DigitalInput BOTTOM_SENSOR;
  private DigitalInput TOP_SENSOR;
  private WarriorSparkMax ELEVATOR_MOTOR;
  private RelativeEncoder ELEVATOR_ENCODER;
  private SysIdRoutine routine = new SysIdRoutine(new SysIdRoutine.Config(),
   new SysIdRoutine.Mechanism((voltage)->this.setVoltage(voltage.in(Volts)), null, this));
  public Elevator() {
    TOP_SENSOR = new DigitalInput(Constants.ElevatorConstants.TOP_SENSOR_ID);
    BOTTOM_SENSOR = new DigitalInput(Constants.ElevatorConstants.BOTTOM_SENSOR_ID);
    ELEVATOR_MOTOR = new WarriorSparkMax(Constants.ElevatorConstants.SPARK_MAX_ID, 
    MotorType.kBrushless,
    false, 
    IdleMode.kCoast);
    ELEVATOR_ENCODER = ELEVATOR_MOTOR.getEncoder();
  }
  public void setVoltage(double voltage){
    ELEVATOR_MOTOR.setVoltage(voltage);
  }
  public void setSpeed(double speed){
    ELEVATOR_MOTOR.set(speed);
  }
  public double getVoltage(){
    return ELEVATOR_MOTOR.getAppliedOutput();
  }
  public boolean getBottomSensor(){
    return !BOTTOM_SENSOR.get();
  }
  public boolean getTopSensor(){
    return !TOP_SENSOR.get();
  }
  public RelativeEncoder getEncoder(){
    return ELEVATOR_ENCODER;
  }
  public double getEncoderPosition(){
    return ELEVATOR_ENCODER.getPosition();
  }
  public void zeroEncoder(){
    ELEVATOR_ENCODER.setPosition(0);
  }
  public Command quasRoutine(SysIdRoutine.Direction direction){
    return routine.quasistatic(direction);
  }
  public Command dynamicRoutine(SysIdRoutine.Direction direction){
    return routine.dynamic(direction);
  }
  public double getVelocity(){
    return ELEVATOR_ENCODER.getVelocity();
  }
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator_Encoder",getEncoderPosition());
    SmartDashboard.putBoolean("Top_Sensor_Value", getTopSensor());
    SmartDashboard.putBoolean("Bottom_Sensor_Value", getBottomSensor());
    SmartDashboard.putNumber("Elevator Voltage",getVoltage());
    SmartDashboard.putNumber("Elevator_Velocity",getVelocity());
    // This method will be called once per scheduler run
  }
}
