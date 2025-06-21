// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.ElevatorMove;
import frc.robot.Commands.ElevatorMove;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.ElevatorIOSparkMax;
import frc.robot.Subsystems.Elevator.levels;

public class RobotContainer {
  public static final Elevator elevator = new Elevator(new ElevatorIOSparkMax());
  public static final CommandXboxController DRIVE_CONTROLLER = new CommandXboxController(0);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Regular Commands
    DRIVE_CONTROLLER.a().onTrue(new ElevatorMove(elevator,levels.L1));
    DRIVE_CONTROLLER.x().onTrue(new ElevatorMove(elevator, levels.L2));
    DRIVE_CONTROLLER.povUp().onTrue(new InstantCommand(()->elevator.zeroEncoder()));
    DRIVE_CONTROLLER.y().onTrue(new ElevatorMove(elevator, levels.L3));
    DRIVE_CONTROLLER.b().onTrue(new ElevatorMove(elevator,levels.L4));
    //SysID Commands
    // DRIVE_CONTROLLER.x().whileTrue(elevator.IDqualatistic(Direction.kForward));
    // DRIVE_CONTROLLER.a().whileTrue(elevator.IDqualatistic(Direction.kReverse));
    // DRIVE_CONTROLLER.y().whileTrue(elevator.IDDynamic(Direction.kForward));
    // DRIVE_CONTROLLER.b().whileTrue(elevator.IDDynamic(Direction.kReverse));
    // DRIVE_CONTROLLER.povDown().whileTrue(new StartEndCommand(()->elevator.setSpeed(-.1),()->elevator.setSpeed(0),elevator));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
