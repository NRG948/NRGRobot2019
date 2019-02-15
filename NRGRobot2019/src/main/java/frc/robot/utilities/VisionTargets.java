/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

import java.util.ArrayList;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class VisionTargets {
    private final String[] NO_TARGETS = new String[0];
    private ArrayList<Target> targets;

    public void update(){
        String[] targetsJson = SmartDashboard.getStringArray("Vision/targets", NO_TARGETS);
        ArrayList<Target> newTargets = new ArrayList<Target>(); 
        GsonBuilder builder = new GsonBuilder();
        Gson gson = builder.create();
        for(int i = 0; i < targetsJson.length; i++){
            newTargets.add(gson.fromJson(targetsJson[i], Target.class));
        }
    }
}


