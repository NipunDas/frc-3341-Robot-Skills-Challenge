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
    addCommands(new DriveForward(2), 
                new AlignBall(true), //turn clockwise for 27 degrees 
                new DriveForward(1.7018),
                new AlignBall(false), //turn counterclockwise for 98 degrees
                new DriveForward(2.413), 
                new AlignBall(true), //turn clockwise for 71 degrees
                new DriveForward(5.2)
              );
  }
}
