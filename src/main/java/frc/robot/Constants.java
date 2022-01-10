/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    // Stores joystick port numbers
    public static final int ID_LEFT_FRONT = 7; //10;
    public static final int ID_RIGHT_FRONT = 4; //15;
    public static final int ID_LEFT_REAR = 8; //11;
    public static final int ID_RIGHT_REAR = 6; //16;

    // Stores motor controller ID numbers
    public static final int USB_LEFT_STICK = 0;
    public static final int USB_RIGHT_STICK = 1;
    public static final int USB_CONTROLLER = 2;
    
    public static final int TRIGGER_LEFT_STICK = 1;
    public static final int MIDDLE_BUTTON_LEFT_STICK = 2;
   
   
    public static class DriveConstants {
        
        public static final double DIAMETER = 6.0;
        public static final double CIRCUMFERENCE = Math.PI * DIAMETER;
        public static final double GEAR_RATIO = 8.71;

        public static final double IN_TO_REV_K = 8.71 / (Math.PI * 6.0);
        public static final double REV_TO_IN_K = CIRCUMFERENCE / GEAR_RATIO;

        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kF = 0.0;

        public static final double MIN_OUTPUT = -1.0;
        public static final double MAX_OUTPUT = 1.0;

        public static final double MARGIN = 4;
        public static final double DISTANCE = 48;
        
    }
    
    
}

