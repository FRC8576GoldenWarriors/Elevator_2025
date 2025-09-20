package frc.robot.Subsystems;

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
  private final WarriorSparkMax elevator_Motor;
  private final DigitalInput topSensor;
  private final DigitalInput bottomSensor;
  private final Encoder thruBore;

  public ElevatorIOSparkMax() {
    elevator_Motor =
        new WarriorSparkMax(
            Constants.ElevatorConstants.SPARK_MAX_ID, MotorType.kBrushless, false, IdleMode.kCoast);
    topSensor = new DigitalInput(Constants.ElevatorConstants.TOP_SENSOR_ID);
    bottomSensor = new DigitalInput(Constants.ElevatorConstants.BOTTOM_SENSOR_ID);
    thruBore = new Encoder(4,3);
    thruBore.setDistancePerPulse(1);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.Bottom_Sensor_Value = !bottomSensor.get();
    inputs.Top_Sensor_Value = !topSensor.get();
    inputs.encoderValue = elevator_Motor.getEncoder().getPosition();
    inputs.motorVoltage = elevator_Motor.getAppliedOutput();
    inputs.motorVelocity = elevator_Motor.getEncoder().getVelocity();
    inputs.thruBoreValue = thruBore.getDistance();
    inputs.thruBoreVelocity = thruBore.getRate();
    inputs.motorCurrent = elevator_Motor.getOutputCurrent();
  }

  @Override
  public void setVoltage(double voltage) {
    elevator_Motor.setVoltage(voltage);
  }
  @Override
  public void zeroEncoder(){
    thruBore.reset();
  }
  @Override
  public void setSpeed(double speed){
    elevator_Motor.set(speed);
  }

  
}
