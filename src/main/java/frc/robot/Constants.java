package frc.robot;

public class Constants {
  public class ElevatorConstants {
    public static final int BOTTOM_SENSOR_ID = 9;
    public static final int TOP_SENSOR_ID = 6;
    public static final int SPARK_MAX_ID = 11;
    public static final double kP = 0.075;//12.8;//6.4;//3.2;//1;//4.8; // 4.8//4.6;//4.65//4.0//0.5//4.8;//7.2, 4.8
    public static final double kI = 0.0;
    public static final double kD = 0;//0.03; // 0.04;//0.04//0.03;//0.01
    public static final double kS = 0;//0.011677;//0.011677; // 0.0;
    public static final double kG = 0;//0.0013733; // 0.1;
    public static final double kA = 0;//1.0575 * Math.pow(10, -5); // 0.0;
    public static final double kV = 0;//0.00016184; // 0.0;
    public static final double feetToRot = 1873.666667;//69.2355;
    public static final double L1Position = 1.5*feetToRot;
    public static final double L2Position = 3*feetToRot;
    public static final double L3Position = 4*feetToRot;
    public static final double L4Position = 5.1*feetToRot;
  }
}
