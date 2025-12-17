package frc.robot.Subsystems;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Constants;

public class ElevatorIOSparkMax implements ElevatorIO {
  private final WarriorSparkMax elevator_Motor_1;
  private final WarriorSparkMax elevator_Motor_2;
  private final DigitalInput topSensor;
  private final DigitalInput bottomSensor;
  private final Encoder thruBore;
  REVLibError motorError;

  public ElevatorIOSparkMax() {
    elevator_Motor_1 =
        new WarriorSparkMax(
            Constants.ElevatorConstants.SPARK_MAX_ID_1, MotorType.kBrushless, false, IdleMode.kCoast,60);
    elevator_Motor_2 = new WarriorSparkMax(Constants.ElevatorConstants.SPARK_MAX_ID_2, MotorType.kBrushless, true, IdleMode.kCoast,60);
    // elevator_Motor_2.follow(Constants.ElevatorConstants.SPARK_MAX_ID_1);
    
    topSensor = new DigitalInput(Constants.ElevatorConstants.TOP_SENSOR_ID);
    bottomSensor = new DigitalInput(Constants.ElevatorConstants.BOTTOM_SENSOR_ID);
    thruBore = new Encoder(4,3,true);
    thruBore.setDistancePerPulse(1);
    
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.Bottom_Sensor_Value = !bottomSensor.get();
    inputs.Top_Sensor_Value = !topSensor.get();
    inputs.encoderValue = elevator_Motor_1.getEncoder().getPosition();
    inputs.motorVoltage = elevator_Motor_1.getAppliedOutput();
    inputs.motorVelocity = elevator_Motor_1.getEncoder().getVelocity();
    inputs.motorVoltage2 = elevator_Motor_2.getAppliedOutput();
    inputs.motorVelocity2 = elevator_Motor_2.getEncoder().getVelocity();
    inputs.thruBoreValue = thruBore.getDistance();
    inputs.thruBoreVelocity = thruBore.getRate();
    inputs.motorCurrent = elevator_Motor_1.getOutputCurrent();
    inputs.motorCurrent2 = elevator_Motor_2.getOutputCurrent();

  }

  @Override
  public void setVoltage(double voltage) {
    elevator_Motor_1.setVoltage(voltage);
    elevator_Motor_2.setVoltage(voltage);
  }
  @Override
  public void zeroEncoder(){
    thruBore.reset();
  }
  @Override
  public void setSpeed(double speed){
    elevator_Motor_1.set(speed);
    elevator_Motor_2.set(speed);
  }

  
}
