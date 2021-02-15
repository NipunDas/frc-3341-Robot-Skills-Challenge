// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.I2C;
import java.nio.ByteBuffer;

public class Ultrasonic extends SubsystemBase {
  //declaring I2C object (the ultrasonic sensor)
  private I2C ultrasonic;
  //default address for the MB1222 ultrasonic sensor
  private int address = 224;
  //ByteBuffer for storing readings
  ByteBuffer buffer = ByteBuffer.allocate(2);

  /** Creates a new Ultrasonic. */
  public Ultrasonic() {
    ultrasonic = new I2C(I2C.Port.kOnboard, address);
  }

  //returns distance in cm
  public double getDistance() {
    ultrasonic.read(225, 2, buffer);
    return buffer.getDouble(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ultrasonic.write(address, 81); //gets a distance reading
    System.out.println(getDistance()); //test print statement
  }
}
