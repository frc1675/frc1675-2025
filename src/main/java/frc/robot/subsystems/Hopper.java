// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {

    /** Creates a new Hopper. */
    public Hopper() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    enum hopperState {
      ON,
      OFF
    }

    hopperState state = hopperState.OFF;

    public void changeState(boolean epicName){

    }
    
    public  getState(){
      switch(state);
      case ON: return true;
      break;
      case OFF: return false;
      break;
    }
    
}
