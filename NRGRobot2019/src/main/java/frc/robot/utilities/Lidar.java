package frc.robot.utilities;

import java.io.IOException;
import java.nio.ByteOrder;

import edu.wpi.first.wpilibj.I2C;

public class Lidar {
	//constants
	protected final int address = 0x52;
	private final int SYSRANGE_START = 0x00;
	private final int SYSTEM_INTERRUPT_CLEAR = 0x0B;
	private final int RESULT_RANGE_STATUS = 0x14;
	private final int RESULT_INTERRUPT_STATUS = 0x13;
	private int ioTimeout;
	private int stopVariable = 0;
    private I2C sensor; 

    public Lidar(I2C.Port port) {
		ioTimeout = 0;
        sensor = new I2C(port, this.address);
		
        //LIDAR setup - to be able to access the lidar sensor
        sensor.write((byte) 0x88, (byte) 0x00);
		sensor.write((byte) 0x80, (byte) 0x01);
		sensor.write((byte) 0xFF, (byte) 0x01);
		sensor.write((byte) 0x00, (byte) 0x00);
		this.stopVariable = this.readU8(this.sensor, 0x91);
		sensor.write((byte) 0x00, (byte) 0x01);
		sensor.write((byte) 0xFF, (byte) 0x00);
		sensor.write((byte) 0x80, (byte) 0x00);
		System.out.println("Lidar Constructor");
    }

    private int readU8(I2C sensor, int register) {
        //this method reads an unsigned byte from the device
		int result = 0;
        byte[] buffer = new byte[1];
        sensor.read(register, 1, buffer);
        result = buffer[0];
		return result; // & 0xFF;
    }
    
    private int readU16BE(I2C sensor, int register) {
        //method reads 2 bytes of BIG ENDIANS
		return readU16(sensor, register, ByteOrder.BIG_ENDIAN);
	}

	private int readU16(I2C sensor, int register, ByteOrder by) {
        //helper method for method above
		int hi = readU8(sensor, register);
		int lo = readU8(sensor, register + 1);
		return ((by ==  ByteOrder.BIG_ENDIAN) ? (hi << 8) + lo : (lo << 8) + hi); // & 0xFFFF;
	}

	public int range() {
		System.out.println("Range Started");
		sensor.write((byte)0x80, (byte)0x01);
		sensor.write((byte)0xFF, (byte)0x01);
		sensor.write((byte)0x00, (byte)0x00);
		sensor.write((byte)0x91, (byte)this.stopVariable);
		sensor.write((byte)0x00, (byte)0x01);
		sensor.write((byte)0xFF, (byte)0x00);
		sensor.write((byte)0x80, (byte)0x00);
		sensor.write((byte)SYSRANGE_START, (byte)0x01);

		try {
			//java.util.concurrent.TimeUnit.MILLISECONDS.sleep(200);
			while ((this.readU8(this.sensor, SYSRANGE_START) & 0x01) > 0) {}	
			System.out.println("Done checking SYSRANGE");
			while ((this.readU8(this.sensor, RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
				System.out.println(this.readU8(this.sensor, RESULT_INTERRUPT_STATUS) & 0x07);
			}
			System.out.println("RESULT_INTERRUPT_STATUS no longer 0");
		} catch(Exception e) {
			System.out.println("not working");
		}
		// long start = System.currentTimeMillis();
		// }
		// start = System.currentTimeMillis();
		// 	if (this.ioTimeout > 0 && ((System.currentTimeMillis() - start) / 1_000) >= this.ioTimeout) {
		// 		throw new RuntimeException("Timeout waiting for VL53L0X!");
		// 	}
		// }
		
		System.out.println("hi: " + readU8(this.sensor, RESULT_RANGE_STATUS) + " lo: " + readU8(this.sensor, RESULT_RANGE_STATUS + 1));
		//System.out.println(this.readU16BE(this.sensor, RESULT_RANGE_STATUS + 10));
		int rangeMm = this.readU16BE(this.sensor, RESULT_RANGE_STATUS + 10);
		sensor.write((byte)SYSTEM_INTERRUPT_CLEAR, (byte)0x01);
		return rangeMm;
	}

}