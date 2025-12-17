package frc.robot.Subsystems;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  default void updateInputs(ElevatorIOInputs inputs) {}

  @AutoLog
  class ElevatorIOInputs {
    public boolean Bottom_Sensor_Value = false;
    public boolean Top_Sensor_Value = false;
    public double encoderValue = 0.0;
    public double motorVoltage = 0.0;
    public double motorVelocity = 0.0;
    public double motorVoltage2 = 0.0;
    public double motorVelocity2 = 0.0;
    public double thruBoreValue = 0.0;
    public double thruBoreVelocity = 0.0;
    public double motorCurrent = 0.0;
    public double motorCurrent2 = 0.0;
  }

  default void setVoltage(double voltage) {}
  default void zeroEncoder(){}
  default void setSpeed(double speed){}
}
