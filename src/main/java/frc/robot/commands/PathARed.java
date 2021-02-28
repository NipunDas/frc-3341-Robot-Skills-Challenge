// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathARed extends SequentialCommandGroup {
  /** Creates a new PathARed. */
  public PathARed() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveForward(1), 
                new LineUp(),
                new DriveForward(1),
                new AlignBall(true), //turn clockwise for 27 degrees 
                new DriveForward(0.5),
                new LineUp(),
                new DriveForward(1.25), //second ball done
                new AlignBall(false), //turn counterclockwise for 98 degrees
                new DriveForward(1.2),
                new LineUp(),
                new DriveForward(1.5),
                new TimedTurn(true, 0.7),
                new DriveForward(4)
              );
  }
}
