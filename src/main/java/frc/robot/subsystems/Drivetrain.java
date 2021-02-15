// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private AHRS navx = new AHRS(SPI.Port.kMXP);
  private WPI_TalonSRX leftTalon = new WPI_TalonSRX(Constants.leftDrivePort);
  private WPI_TalonSRX rightTalon = new WPI_TalonSRX(Constants.rightDrivePort);
  private WPI_TalonSRX flywheel1 = new WPI_TalonSRX(Constants.wheel1Port);
  private WPI_TalonSRX flywheel2 = new WPI_TalonSRX(Constants.wheel2Port);
  private static Drivetrain drive;

  private double kTicksToInches = 0.152 * Math.PI * (1.0/4096.0);

  public Drivetrain() {
    //setting up left and right talons and encoders
    leftTalon.configFactoryDefault();
    leftTalon.setInverted(false);
    leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    rightTalon.configFactoryDefault();
    rightTalon.setInverted(true);
    rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    flywheel1.configFactoryDefault();
    flywheel2.configFactoryDefault();
    flywheel1.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    flywheel2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    resetEncoders();

    //resetting the gyro
    navx.reset();
  }

  public Drivetrain getInstance() {
    if (drive == null) {
      drive = new Drivetrain();
    }
    return drive;
  }

  public void tankDrive(double leftPow, double rightPow) {
    leftTalon.set(ControlMode.PercentOutput, leftPow);
    rightTalon.set(ControlMode.PercentOutput, rightPow);
  }

  public void resetEncoders() {
    leftTalon.setSelectedSensorPosition(0, 0, 10);
    rightTalon.setSelectedSensorPosition(0, 0, 10);
  }

  public double getEncoderDistance() {
    return (leftTalon.getSelectedSensorPosition(0) + rightTalon.getSelectedSensorPosition(0)) * -0.5 * kTicksToInches;
  }

  public void resetNavx() {
    navx.reset();
  }

  public double getAngle() {
    return navx.getAngle()%360;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tankDrive(RobotContainer.returnLeftJoy().getY(), RobotContainer.returnRightJoy().getY());
    System.out.println(getEncoderDistance());
    if (RobotContainer.returnLeftJoy().getRawButton(8)) {
      flywheel1.set(ControlMode.PercentOutput, 0.5);
      flywheel2.set(ControlMode.PercentOutput, 0.5);
    } else {
      flywheel1.set(ControlMode.PercentOutput, 0);
      flywheel2.set(ControlMode.PercentOutput, 0);
    }
  }
}
