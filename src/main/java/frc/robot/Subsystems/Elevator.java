// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  // private DigitalInput BOTTOM_SENSOR;
  // private DigitalInput TOP_SENSOR;
  // private WarriorSparkMax ELEVATOR_MOTOR;
  // private RelativeEncoder ELEVATOR_ENCODER;
  // private SysIdRoutine routine =
  //     new SysIdRoutine(
  //         new SysIdRoutine.Config(),
  //         new SysIdRoutine.Mechanism((voltage) -> this.setVoltage(voltage.in(Volts)), null,
  // this));

  private ElevatorIO io;

  private double PID_Voltage;
  private double FF_Voltage;
  private double Input_Voltage;
  private ProfiledPIDController PID;
  private ElevatorFeedforward FEED_FORWARD;
  private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private double wantedPosition;
  private double currentPosition;

  public Elevator(ElevatorIO io) {
    // TOP_SENSOR = new DigitalInput(Constants.ElevatorConstants.TOP_SENSOR_ID);
    // BOTTOM_SENSOR = new DigitalInput(Constants.ElevatorConstants.BOTTOM_SENSOR_ID);
    // ELEVATOR_MOTOR =
    //     new WarriorSparkMax(
    //         Constants.ElevatorConstants.SPARK_MAX_ID, MotorType.kBrushless, false,
    // IdleMode.kCoast);
    // ELEVATOR_ENCODER = ELEVATOR_MOTOR.getEncoder();
    PID =
        new ProfiledPIDController(
            Constants.ElevatorConstants.kP,
            Constants.ElevatorConstants.kI,
            Constants.ElevatorConstants.kD,
            new TrapezoidProfile.Constraints(400, 500));
    PID.setTolerance(15);
    FEED_FORWARD =
        new ElevatorFeedforward(
            Constants.ElevatorConstants.kS,
            Constants.ElevatorConstants.kG,
            Constants.ElevatorConstants.kV,
            Constants.ElevatorConstants.kA);
    this.io = io;
    currentPosition = inputs.encoderValue;
    wantedPosition = -1;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    currentPosition = inputs.encoderValue;
    // prevent elevator from moving after instantiated
    if (wantedPosition >= 0&&(getEncoderPosition()<220||!getTopSensor())) {
      PID_Voltage = PID.calculate(currentPosition, wantedPosition);
      FF_Voltage = FEED_FORWARD.calculate(10, 15);
      Input_Voltage = PID_Voltage + FF_Voltage;
      io.setVoltage(Input_Voltage);
    } else {
      io.setVoltage(0);
    }
    SmartDashboard.putBoolean("Bottom_Sensor", getBottomSensor());
  }

  public void setWantedPosition(double position) {
    wantedPosition = position; // rotations
  }

  public void resetPID() {
    PID.reset(inputs.encoderValue, 0);
  }

  // public void setVoltage(double voltage) {
  //   ELEVATOR_MOTOR.setVoltage(voltage);
  // }

  // public void setSpeed(double speed) {
  //   ELEVATOR_MOTOR.set(speed);
  // }

 
  public double getVoltage() {
    return inputs.motorVoltage;
    // return ELEVATOR_MOTOR.getAppliedOutput();
  }

  public boolean getBottomSensor() {
    return inputs.Bottom_Sensor_Value;
    // return !BOTTOM_SENSOR.get();
  }

  public boolean getTopSensor() {
    return inputs.Top_Sensor_Value;
    // return !TOP_SENSOR.get();
  }

  // public RelativeEncoder getEncoder() {
  //   return ELEVATOR_ENCODER;
  // }

  public double getEncoderPosition() {
    return inputs.encoderValue;
    // return ELEVATOR_ENCODER.getPosition();
  }

  public void zeroEncoder() {
    io.setEncoderPosition(0);
  }

  // public Command quasRoutine(SysIdRoutine.Direction direction) {
  //   return routine.quasistatic(direction);
  // }

  // public Command dynamicRoutine(SysIdRoutine.Direction direction) {
  //   return routine.dynamic(direction);
  // }

  public double getVelocity() {
    return inputs.motorVelocity;
  }

  // @Override
  // public void periodic() {
  //   SmartDashboard.putNumber("Elevator_Encoder", getEncoderPosition());
  //   SmartDashboard.putBoolean("Top_Sensor_Value", getTopSensor());
  //   SmartDashboard.putBoolean("Bottom_Sensor_Value", getBottomSensor());
  //   SmartDashboard.putNumber("Elevator Voltage", getVoltage());
  //   SmartDashboard.putNumber("Elevator_Velocity", getVelocity());
  //   // This method will be called once per scheduler run
  // }
}
