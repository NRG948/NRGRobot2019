/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import java.util.ArrayList;

/**
 * Add your docs here.
 */
public class ColorSensor {

public static ColorSensorLink link;
ArrayList<Block> colorSensorData = new ArrayList<Block>();
Block currColorSensorBlock;
private int blockNumCounter;

    public ColorSensor(ColorSensorLink link){
        this.link = link;
    }
    private int byteToInt(int binary){ // Changes 1 Byte into Int form
		int x = 0;
		for(int i = 0; i < 8; i++) {
			if(binary % 10 == 1) {
				x += Math.pow(2, i);
			}
			binary /= 10;
		}
        return x;
    }

    public int getRed(){ // Returns red values of color sensor
        return byteToInt(currColorSensorBlock.ColorSensorValues[0]); 
    }
    public int getGreen(){ // returns green vlaues
        return byteToInt(currColorSensorBlock.ColorSensorValues[1]);
    }
    public int getBlue(){ // returns blue values
        return byteToInt(currColorSensorBlock.ColorSensorValues[2]);
    }
    public int getAlpha(){ // returns alpha values
        return byteToInt(currColorSensorBlock.ColorSensorValues[3]);
    }
    public void updateColorSensor(){
        currColorSensorBlock = new Block(this.link);
        colorSensorData.add(currColorSensorBlock);
    }
    public ArrayList<Block> getBlockArray(){
        return colorSensorData;
    }
    public void setBlock(Block b){
        currColorSensorBlock = b;
    }
    public Block getBlock(){
        return currColorSensorBlock;
    }

    public class Block {

        int[] ColorSensorValues;
    
        public Block( ColorSensorLink link){
            this.ColorSensorValues = link.getIntArray(); 
        }
        
    }
}

