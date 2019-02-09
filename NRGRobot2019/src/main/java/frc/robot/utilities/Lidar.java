package frc.robot.utilities;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.wpilibj.I2C;

public class Lidar {
    protected final int address = 0x29;
    private I2C sensor; 

    public Lidar(I2C.Port port) {
        sensor = new I2C(port, this.address);

        //LIDAR setup - to be able to access the lidar sensor
        sensor.write((byte) 0x88, (byte) 0x00);
		sensor.write((byte) 0x80, (byte) 0x01);
		sensor.write((byte) 0xFF, (byte) 0x01);
		sensor.write((byte) 0x00, (byte) 0x00);
		sensor.write((byte) 0x00, (byte) 0x01);
		sensor.write((byte) 0xFF, (byte) 0x00);
        sensor.write((byte) 0x80, (byte) 0x00);
    }
    public static int readU8(I2C sensor, int reg) {
		int result = 0;
        byte[] buffer = new byte[1];
        result = sensor.read(reg, 1, buffer[0]);
		return result; // & 0xFF;
    }
    
    public static int readU16BE(I2C sensor, int register) throws IOException{
		return readU16(sensor, register, ByteOrder.BIG_ENDIAN);
	}

	public static int readU16(I2C sensor, int register, ByteOrder by) throws IOException {
		int hi = readU8(sensor, register);
		int lo = readU8(sensor, register + 1);
		return ((by ==  ByteOrder.BIG_ENDIAN) ? (hi << 8) + lo : (lo << 8) + hi); // & 0xFFFF;
	}

}