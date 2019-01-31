/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;


import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

/**
 * Add your docs here.
 */
public class ColorSensor {

    I2C i2c;
    int deviceAddress;

    byte CLR_INT = (byte) 0xE0; 
    byte REG_BLOCK_READ = (byte) 0xD0;



    public ColorSensor(Port port, int deviceAddress){
        i2c = new I2C(port, deviceAddress);
        this.deviceAddress = deviceAddress;
    }
    public void clearInterrupt(){
        byte[] b = new byte[]{CLR_INT};
        i2c.writeBulk(b, b.length);
    }
    public Color readRGB(){
        byte[] b = new byte[]{REG_BLOCK_READ};
        byte[] data = new byte[8];
        i2c.transaction(b, b.length, data, data.length);
        Color color = new Color();
        color.red = data[1] * 256 + data[0];
        color.green = data[3] * 256 + data[2];
        color.blue = data[5] * 256 + data[4];
        color.alpha = data[7] * 256 + data[6];
        return color;

    }
    public class Color{
        public int red;
        public int green;
        public int blue;
        public int alpha;

    }
 }

