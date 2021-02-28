// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.RobotContainer;

public class AlignBall extends CommandBase {
  /** Creates a new AlignBall. */

  private Limelight li;
  private double x;
  private boolean right;
  private double power;

  public AlignBall(boolean turnRight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.returnDrive().getInstance());
    right = turnRight;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    li = new Limelight();
    x = li.getX();
    System.out.println(x);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    x = li.getX();
    if (x == 0) {
      power = 0.4;
    } else {
      power = 0.4;//power = 0.02 * Math.abs(x);
    }
    if (right) {
      RobotContainer.returnDrive().tankDrive(power, -power);
    } else {
      RobotContainer.returnDrive().tankDrive(-power, power);
    }
    RobotContainer.returnDrive().intakeBalls();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.returnDrive().tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (x != 0 && Math.abs(x) < 2);
  }
}
