/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;

public class I2Cwrapper implements ColorSensorLink {
	protected final static int CMD = 0x80;
	protected final static int MULTI_BYTE_BIT = 0x20;



	protected final static int ENABLE_REGISTER  = 0x00;
	protected final static int ATIME_REGISTER   = 0x01;
	protected final static int PPULSE_REGISTER  = 0x0E;
	
	protected final static int ID_REGISTER     = 0x12;
	protected final static int CDATA_REGISTER  = 0x14;
	protected final static int RDATA_REGISTER  = 0x16;
	protected final static int GDATA_REGISTER  = 0x18;
	protected final static int BDATA_REGISTER  = 0x1A;
	protected final static int PDATA_REGISTER  = 0x1C;
	protected final static int PON   = 0b00000001;
	protected final static int AEN   = 0b00000010;
	protected final static int PEN   = 0b00000100;

	private final double integrationTime = 10;

	private I2C i2c;
	private Port i2cPort;
	private int deviceAddress;

	private ByteBuffer buffy = ByteBuffer.allocate(8);
	public int red = 0, green = 0, blue = 0, prox = 0;

	public I2Cwrapper(Port port, int deviceAddress) {
		i2cPort = port;
		this.deviceAddress = deviceAddress;
		i2c = new I2C(i2cPort, this.deviceAddress);
		i2c.write(CMD | 0x00, PON | AEN | PEN);
    
		i2c.write(CMD | 0x01, (int) (256-integrationTime/2.38)); //configures the integration time (time for updating color data)
		i2c.write(CMD | 0x0E, 0b1111);
	}

	public int getWord() {
		byte[] c = new byte[2];
		i2c.readOnly(c, 2);
		int w = ((c[1] & 0xff) << 8) + (c[0] & 0xff);
        return w;
	}

    public int[] readColorSensor() { // covert byte array to int array
		buffy.clear();
		int[] c = new int[4];
		if(i2c.read(CMD | MULTI_BYTE_BIT | RDATA_REGISTER, 8, buffy)){
			System.out.println("YES");
		} else {
			System.out.println("NO");
		}
		
		red = buffy.getShort(0) & 0xFFFF;
		green = buffy.getShort(2) & 0xFFFF;
		blue = buffy.getShort(4) & 0xFFFF; 
		prox = buffy.getShort(6) & 0xFFFF; 
		c[0] = red;
		c[1] = green;
		c[2] = blue;
		c[3] = prox;
		return c;
    }

	public void send(byte[] data) {
		i2c.writeBulk(data);
	}
}

