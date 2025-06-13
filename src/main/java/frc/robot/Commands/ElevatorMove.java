// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorMove extends Command {
  /** Creates a new ElevatorMove. */
  private Elevator elevator;
  private double position;
  private ProfiledPIDController PID;
  // private PIDController PID;
  private ElevatorFeedforward FEED_FORWARD;
  private double PID_VOLTAGE;
  private double FEEDFORWARD_VOLTAGE;
  private double INPUT_VOLTAGE;
  public ElevatorMove(Elevator elevator, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.position = position*69.2355;//convert to rotations
    PID = new ProfiledPIDController(Constants.ElevatorConstants.kP, 
    Constants.ElevatorConstants.kI,
    Constants.ElevatorConstants.kD, 
    new TrapezoidProfile.Constraints(400,500));
    PID.setTolerance(15);
    // PID = new PIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI, Constants.ElevatorConstants.kD);
    // PID.setTolerance(10);
    FEED_FORWARD = new ElevatorFeedforward(Constants.ElevatorConstants.kS, Constants.ElevatorConstants.kG, Constants.ElevatorConstants.kV,Constants.ElevatorConstants.kA);
    SmartDashboard.putNumber("PID Voltage",PID_VOLTAGE);
    SmartDashboard.putNumber("FeedForward Voltage",FEEDFORWARD_VOLTAGE);
    SmartDashboard.putNumber("Input voltage",INPUT_VOLTAGE);
    SmartDashboard.putNumber("Error",PID.getPositionError());
    PID.reset(new State(elevator.getEncoderPosition(),elevator.getVelocity()));
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    PID.reset(elevator.getEncoderPosition(),0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    PID_VOLTAGE = PID.calculate(elevator.getEncoderPosition(),position);
    FEEDFORWARD_VOLTAGE = FEED_FORWARD.calculate(10,15);
    INPUT_VOLTAGE = PID_VOLTAGE+FEEDFORWARD_VOLTAGE;
    if(elevator.getEncoderPosition()<213||!elevator.getTopSensor()){
    elevator.setVoltage(INPUT_VOLTAGE);
    }
    else{
      elevator.setVoltage(0.0);
      end(false);
    }
    SmartDashboard.putNumber("PID Voltage",PID_VOLTAGE);
    SmartDashboard.putNumber("FeedForward Voltage",FEEDFORWARD_VOLTAGE);
    SmartDashboard.putNumber("Input voltage",INPUT_VOLTAGE);
    SmartDashboard.putNumber("SetPoint",position);
    SmartDashboard.putNumber("Error",PID.getPositionError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setVoltage(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
