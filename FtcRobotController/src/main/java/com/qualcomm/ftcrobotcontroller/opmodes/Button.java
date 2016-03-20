package com.qualcomm.ftcrobotcontroller.opmodes;

import java.nio.ByteBuffer;

/**
 * Handles buttons for each joystick. Allows you to implement button actions in a clean way at the
 * start of the loop.
 *
 * copied from ftc_app/FtcRobotController/src/main/java/com/qualcomm/ftcrobotcontroller/opmodes/Button.java
 * https://github.com/The-Knights-of-Ni/ftc_app/blob/master/FtcRobotController/src/main/java/com/qualcomm/ftcrobotcontroller/opmodes/Button.java
 */
public class Button //Might want to extend gamepad instead of passing to it later. I think the opmode structure makes this annoying though.
{
    //TODO: Tap - similar to press, maybe not needed (Press that only triggers once)
    //TODO: Combination - true if all buttons pressed
    //TODO: Release - true when released
    //TODO: Trigger -> Button option
    //TODO: Move this into a reasonable package
    private int currentByte = 0; //Set these in update function
    private int previousByte = 0;//Can assume this is 0 at init.
    final int offset = 40;//Location of the Button storage integer in Gamepad.java

    /**
     * Storage for locations and statuses of each button we need. You can use .prevStatus = true if
     * you want the toggle/other function to start as true.
     */
    public enum Buttons {
        LEFT_STICK_BUTTON (15, false), //TODO: Make status into a byte that will tell us more information if needed.
        RIGHT_STICK_BUTTON (14, false),
        DPAD_UP (13, false),
        DPAD_DOWN (12, false),
        DPAD_LEFT (11, false),
        DPAD_RIGHT (10, false),
        A (9, false),
        B (8, false),
        X (7, false),
        Y (6, false),
        LOGITECH (5, false),
        START (4, false),
        BACK (3, false),
        LEFT_BUMPER (2, false),
        RIGHT_BUMPER (1, false);
        public int location;
        public boolean prevStatus;//This should let us change a default if we want to.
        Buttons(int location, boolean prevStatus)
        {
            this.location = location;
            this.prevStatus = prevStatus;
        }
    }

    //Check the state of a button inside of the button integer.
    private static boolean getState(int btn, int bits) //Might be better to put this in the enum (in a separate file) and import that.
    {
        if(((bits >>> (btn-1)) & 1) != 0)
            return true;
        else
            return false;
    }

    //============================ Start of actual button uses ================================

    /**
     * Assign a button to press. Leave in loop.
     * @param btn A button on the joystick.
     * @return true while the button is held, false while it's not
     */
    public boolean press(Buttons btn)
    {
        if(getState(btn.location, currentByte))
            return true;
        return false;
    }

    /**
     * Assign a button to press. Leave in loop.
     * @param btn A button on the joystick.
     * @return true when the button is first pressed, false while it's not
     */
    public boolean singlePress(Buttons btn)
    {
        return (getState(btn.location, currentByte) == true) && (getState(btn.location, previousByte) == false);
    }

    /**
     * Assign a button to toggle. Leave in loop.
     * @param btn A button on the joystick.
     * @return true or false depending on state, button switches state.
     */
    public boolean toggle(Buttons btn)
    {
        if((getState(btn.location, currentByte) == true) && (getState(btn.location, previousByte) == false))//If there's a change and it's from low to high, flip it, if not, keep toggle the same.
            btn.prevStatus = !btn.prevStatus;
        return btn.prevStatus;
    }

    /**
     * Not working yet.
     * @param btn A button on the joystick.
     * @return
     */
    private boolean combinationPress(Buttons[] btn) //TODO: Make this a lookup table.
    {
        boolean isEqual = true;
        for(int i = 0; i < btn.length; i++){
            if(!getState(btn[i].location, currentByte))
                isEqual = false;
        }
        return isEqual;
    }

    /**
     * Not yet implemented.
     * @param btn A button on the joystick
     * @return true when the button is released, false when pressed/not pressed.
     */
    private boolean release(Buttons btn)
    {
        if((!getState(btn.location, currentByte)) && (getState(btn.location, currentByte) != getState(btn.location, previousByte)))
            return false;
        return true;
    }

    /**
     * Update function that refreshes the button storage integers and makes it work.
     * To implement, call object.updateButtons(gamepad#.toByteArray()) in a try/catch inside the loop.
     * @param joystick The byte array from a gamepad.
     */
    public void updateButtons(byte[] joystick) //TODO: Add lookup method that checks if currentByte == sum of a button combination and then makes it 0 if needed.
    {
        ByteBuffer stick = ByteBuffer.allocate(45);
        stick.put(joystick);
        previousByte = currentByte;
        currentByte = stick.getInt(offset);
    }



    /*
    (Saving for EN - This is how I built the class)
    Should take a byte array from the joystick, only grab the bytes we need (the button + triggers)
    45 bytes per controller
    Testing reveals that
    At getInt(39):
    leftstick = 64
    rightstick = 32
    dpadup = 16
    dpaddown = 8
    dpadleft = 4
    dpadright = 2
    a = 1
    At getInt(38) we only see right trigger
    At getInt(40): all buttons are registered.
          1  var1.put(this.getRobocolMsgType().asByte());
          2  var1.putShort((short)42);
          1  var1.put((byte)2);
          4  var1.putInt(this.id);
          8  var1.putLong(this.timestamp).array();
          4  var1.putFloat(this.left_stick_x).array();
          4  var1.putFloat(this.left_stick_y).array();
          4  var1.putFloat(this.right_stick_x).array();
          4  var1.putFloat(this.right_stick_y).array();
            TODO: 16 + 8 + 4 + 1 + 2 = 30 bytes until triggers. (Not a todo, I just wanted the color)
          4  var1.putFloat(this.left_trigger).array();
          4  var1.putFloat(this.right_trigger).array();
            TODO: 38 bytes until button int. (Not a todo)
          1.0  int var4 = (var2 << 1) + (this.left_stick_button?1:0); -> They're storing all the 0/1 buttons in a single int (4 bytes = 32 bits).
          1.1  var4 = (var4 << 1) + (this.right_stick_button?1:0);
          1.2  var4 = (var4 << 1) + (this.dpad_up?1:0);
          1.3  var4 = (var4 << 1) + (this.dpad_down?1:0);
          1.4  var4 = (var4 << 1) + (this.dpad_left?1:0);
          1.5  var4 = (var4 << 1) + (this.dpad_right?1:0);
          1.6  var4 = (var4 << 1) + (this.a?1:0);
          1.7  var4 = (var4 << 1) + (this.b?1:0);
          2.0  var4 = (var4 << 1) + (this.x?1:0);
          2.1  var4 = (var4 << 1) + (this.y?1:0);
          2.2  var4 = (var4 << 1) + (this.guide?1:0);
          2.3  var4 = (var4 << 1) + (this.start?1:0);
          2.4  var4 = (var4 << 1) + (this.back?1:0);
          2.5  var4 = (var4 << 1) + (this.left_bumper?1:0);
          2.6  var4 = (var4 << 1) + (this.right_bumper?1:0); not sure why they allocate so much.
          4  var1.putInt(var4);
            var1.put(this.user);
     */
}