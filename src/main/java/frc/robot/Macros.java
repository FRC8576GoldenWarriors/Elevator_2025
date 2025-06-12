package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.Elevator;

public class Macros {
    public static SequentialCommandGroup elevatorReset(Elevator elevator){
        return new SequentialCommandGroup(
            new StartEndCommand(()->elevator.setSpeed(-.1), ()->elevator.setSpeed(0), elevator).until(()->elevator.getBottomSensor()),
            new WaitCommand(0.3),
            new StartEndCommand(()->elevator.setSpeed(0), ()->elevator.setSpeed(0), elevator).withTimeout(0.2),
            new InstantCommand(()->elevator.zeroEncoder())
        );
    }
}
