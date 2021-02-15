// // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class TurnRobot extends CommandBase {

  double targetAngle, error;
  double kP = 0.01;

  /** Creates a new TurnRobot. */
  public TurnRobot(double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.returnDrive().getInstance());
    targetAngle = angle;
    error = targetAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.returnDrive().resetNavx();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = targetAngle - RobotContainer.returnDrive().getAngle();
    RobotContainer.returnDrive().tankDrive(kP * error, -kP * error);
    System.out.println("Angle: " + RobotContainer.returnDrive().getAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.returnDrive().tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(error) <= 2) {
      return true;
    }
    return false;
  }
}
