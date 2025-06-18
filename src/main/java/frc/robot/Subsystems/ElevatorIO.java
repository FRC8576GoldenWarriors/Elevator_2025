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
  }

  default void setVoltage(double voltage) {}
  default void setEncoderPosition(double position){}
  default void setSpeed(double speed){}
}
