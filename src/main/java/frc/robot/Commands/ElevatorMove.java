// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.Elevator.levels;

/* You should consider using the more terse Command factories API instead
https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorMove extends Command {
  /** Creates a new ElevatorMove. */
  private Elevator elevator;

  private levels level;

  //   private ProfiledPIDController PID;
  //   // private PIDController PID;
  //   private ElevatorFeedforward FEED_FORWARD;
  //   private double PID_VOLTAGE;
  //   private double FEEDFORWARD_VOLTAGE;
  //   private double INPUT_VOLTAGE;

  public ElevatorMove(Elevator elevator, Elevator.levels level) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;

    this.level = level; // convert to rotations
    // PID =
    //     new ProfiledPIDController(
    //         Constants.ElevatorConstants.kP,
    //         Constants.ElevatorConstants.kI,
    //         Constants.ElevatorConstants.kD,
    //         new TrapezoidProfile.Constraints(400, 500));
    // PID.setTolerance(15);
    // PID = new PIDController(Constants.ElevatorConstants.kP, Constants.ElevatorConstants.kI,
    // Constants.ElevatorConstants.kD);
    // // PID.setTolerance(10);
    // FEED_FORWARD =
    //     new ElevatorFeedforward(
    //         Constants.ElevatorConstants.kS,
    //         Constants.ElevatorConstants.kG,
    //         Constants.ElevatorConstants.kV,
    //         Constants.ElevatorConstants.kA);
    // SmartDashboard.putNumber("PID Voltage", PID_VOLTAGE);
    // SmartDashboard.putNumber("FeedForward Voltage", FEEDFORWARD_VOLTAGE);
    // SmartDashboard.putNumber("Input voltage", INPUT_VOLTAGE);
    // SmartDashboard.putNumber("Error", PID.getPositionError());
    // PID.reset(new State(elevator.getEncoderPosition(), elevator.getVelocity()));
    elevator.resetPID();
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.resetPID();
    elevator.setWantedLevel(level);
    // PID.reset(elevator.getEncoderPosition(), 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // PID_VOLTAGE = PID.calculate(elevator.getEncoderPosition(), position);
    // FEEDFORWARD_VOLTAGE = FEED_FORWARD.calculate(10, 15);
    // INPUT_VOLTAGE = PID_VOLTAGE + FEEDFORWARD_VOLTAGE;
    // if (elevator.getEncoderPosition() < 213 || !elevator.getTopSensor()) {
    //   elevator.setVoltage(INPUT_VOLTAGE);
    // } else {
    //   elevator.setVoltage(0.0);
    //   end(false);
    //   return;
    // }
    // SmartDashboard.putNumber("PID Voltage", PID_VOLTAGE);
    // SmartDashboard.putNumber("FeedForward Voltage", FEEDFORWARD_VOLTAGE);
    // SmartDashboard.putNumber("Input voltage", INPUT_VOLTAGE);
    // SmartDashboard.putNumber("SetPoint", position);
    // SmartDashboard.putNumber("Error", PID.getPositionError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setWantedLevel(levels.Idle);
    // elevator.setVoltage(0.0);
    return;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
