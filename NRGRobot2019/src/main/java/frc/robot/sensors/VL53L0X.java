/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SendableBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

/**
 * Add your docs here.
 */
public class VL53L0X extends SendableBase {
    private static final int DEVICE_ADDRESS = 0x52;
    private static final int SYSRANGE_START_REGISTER = 0x00;
    private static final int SYSTEM_INTERRUPT_CLEAR_REGISTER = 0x0B;
    private static final int SYSTEM_INTERRUPT_STATUS_REGISTER = 0x13;
    private static final int RANGE_STATUS_REGISTER = 0x14;
    private static final int SYSRANGE_MODE_BACKTOBACK = 0x02;
    private I2C i2c;
    private int stopVariable;
    private volatile boolean enabled;
    private int distance;

    public VL53L0X(I2C.Port port) {
        this.i2c = new I2C(port, DEVICE_ADDRESS);
        this.i2c.write(0x88, 0x00);
        i2c.write(0x80, 0x01);
        i2c.write(0xFF, 0x01);
        i2c.write(0x00, 0x00);
        byte[] buffer = new byte[1];
        i2c.read(0x91, buffer.length, buffer);
        stopVariable = buffer[0];
        i2c.write(0x00, 0x01);
        i2c.write(0xFF, 0x00);
        i2c.write(0x80, 0x00);
        setName("VL53L0X");
    }

    private void startMeasurement() {
        i2c.write(0x80, 0x01);
        i2c.write(0xFF, 0x01);
        i2c.write(0x00, 0x00);
        i2c.write(0x91, stopVariable);
        i2c.write(0x00, 0x01);
        i2c.write(0xFF, 0x00);
        i2c.write(0x80, 0x00);
        i2c.write(SYSRANGE_START_REGISTER, SYSRANGE_MODE_BACKTOBACK);
    }

    private boolean isDataReady() {
        byte[] status = new byte[1];
        boolean result = i2c.read(RANGE_STATUS_REGISTER, status.length, status);
        System.out.println("read result is " + result);
        return (status[0] & 0x01) != 0;

    }

    private int getMeasurement() {
        byte[] buffer = new byte[12];
        i2c.read(0x14, buffer.length, buffer);
        return ((buffer[10] & 0xFF) << 8) + (buffer[11] & 0xFF);
    }

    private void clearInterrupt() {
        byte[] buffer = new byte[1];
        do {
            i2c.write(SYSTEM_INTERRUPT_CLEAR_REGISTER, 0x01);
            i2c.write(SYSTEM_INTERRUPT_CLEAR_REGISTER, 0x00);
            i2c.read(SYSTEM_INTERRUPT_STATUS_REGISTER, buffer.length, buffer);
        } while ((buffer[0] & 0x07) != 0);
    }

    private void pollForData() {
        System.out.println("Enter poll for data");
        while (enabled) {
            startMeasurement();
            System.out.println("Measurement started");
            while (!isDataReady()) {
                try {
                    Thread.sleep(1);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            System.out.println("data ready");
            distance = getMeasurement();
            clearInterrupt();
        }
    }

    public void start(){
        enabled = true;
        new Thread(() -> {pollForData();}).start();
    }

    public void stop(){
        enabled = false;
    }

    public double getDistance(){
        return (double)distance;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("VL53L0X");
        builder.addDoubleProperty("Distance", this::getDistance, null);
	}

}
