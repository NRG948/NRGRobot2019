/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class I2Cwrapper implements ColorSensorLink {
	private I2C i2c;
	private Port i2cPort;
	private int deviceAddress;

	public I2Cwrapper(Port port, int deviceAddress) {
		i2cPort = port;
		this.deviceAddress = deviceAddress;
		i2c = new I2C(i2cPort, this.deviceAddress);
	}

	public int getWord() {
		byte[] c = new byte[2];
		i2c.readOnly(c, 2);
		int w = ((c[1] & 0xff) << 8) + (c[0] & 0xff);
        return w;
	}


    public int[] getInt() { // covert byte array to int 
		byte[] c = new byte[5];
        i2c.readOnly(c, 5); // get byte array
        int[] w = new int[5];
        for(int i = 4; i > 0; i--){
            w[i] = (c[i] & 0xff); // save bytes to int array
        }
        w[0] = (c[0] & 0xff);
		return w;
    }
    

	public void send(byte[] data) {
		i2c.writeBulk(data);
	}

}

