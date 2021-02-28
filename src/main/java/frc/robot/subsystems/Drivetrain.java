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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Limelight;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private static Drivetrain drive;
  private WPI_TalonSRX leftTalon = new WPI_TalonSRX(Constants.leftDrivePort);
  private WPI_TalonSRX rightTalon = new WPI_TalonSRX(Constants.rightDrivePort);
  private WPI_TalonSRX intake = new WPI_TalonSRX(Constants.intakePort);
  private WPI_TalonSRX belt = new WPI_TalonSRX(Constants.beltPort);
  private AHRS navx = new AHRS(SPI.Port.kMXP);
  Limelight li = new Limelight();

  private double kTicksToInches = 0.152 * Math.PI * (1.0/4096.0);
  private double threshold = 0, waitTime = 0.1, intakePower = 0.5, beltPower = 0.9, lastTime = Timer.getFPGATimestamp();

  public Drivetrain() {
    //setting up left and right talons and encoders
    leftTalon.configFactoryDefault();
    leftTalon.setInverted(false);
    leftTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

    rightTalon.configFactoryDefault();
    rightTalon.setInverted(true);
    rightTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
    resetEncoders();

    intake.configFactoryDefault();
    intake.setInverted(false);
    belt.configFactoryDefault();
    belt.setInverted(true);

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
    return navx.getAngle()%360.0;
  }

  public void intakeBalls() {
    intake.set(ControlMode.PercentOutput, intakePower);
    if (intake.getSupplyCurrent() > threshold) {
      belt.set(ControlMode.PercentOutput, beltPower);
      lastTime = Timer.getFPGATimestamp();
    } else if (Timer.getFPGATimestamp() - lastTime < waitTime) {
      belt.set(ControlMode.PercentOutput, 0.6);
    } else {
      belt.set(ControlMode.PercentOutput, 0);
    }
  }

  public void stopIntake() {
    intake.set(ControlMode.PercentOutput, 0);
    belt.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    tankDrive(RobotContainer.returnLeftJoy().getY()*0.5, RobotContainer.returnRightJoy().getY()*0.5);
    System.out.println(getAngle());
    intakeBalls();
    if (RobotContainer.returnLeftJoy().getRawButton(10)) {
      belt.set(ControlMode.PercentOutput, 1);
    }
  }
}
