package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.AutoBalanceCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.manipulator.ArmSubsystem;
import frc.robot.subsystems.manipulator.ManipulatorSubsystem;

public class MiddleCubeEngageCS {
    public static Command get(DriveSubsystem driveSubsystem, ManipulatorSubsystem manipulator) {

      
    
        return new SequentialCommandGroup(
                manipulator.getShootCubeCommand(),
                new ProxyCommand(
                    () -> new InstantCommand()),
                new AutoBalanceCommand(driveSubsystem)
                    .alongWith(
                        new WaitUntilCommand(14.9)
                            .andThen(new InstantCommand(() -> driveSubsystem.setParkMode(true)))),
                // new LogCommand("After follow path command, about to run cube hunt."),
                // new CubeHuntCommand(driveSubsystem, () ->
                // manipulator.getRollerSensor()).withTimeout(2.0),
                // new InstantCommand(() -> manipulator.rollerStop()),
                // new InstantCommand(() -> manipulator.grab()),
                new InstantCommand(() -> driveSubsystem.stop()));
          
      }
    
}
