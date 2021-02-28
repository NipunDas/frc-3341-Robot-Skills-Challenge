// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.RobotContainer;

public class LineUp extends CommandBase {

  private Limelight li;
  private double x;
  private double power;
  /** Creates a new LineUp. */
  public LineUp() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.returnDrive().getInstance());
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
    power = 0.3;
    if (x > 0) {
      RobotContainer.returnDrive().tankDrive(power, -power);
    } else {
      RobotContainer.returnDrive().tankDrive(-power, power);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.returnDrive().tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(x) < 4;
  }
}
