package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.ftcrobotcontroller.opmodes.Button;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This program runs a demo TeleOp for the Black Knight
 *
 * There are two controllers: one with drive, and the other with everything else -
 * Main Controller (controller1):
 * - Drive (Left Joystick)
 *
 * Auxiliary Controller (controller2):
 * - Lift (Right Joystick)
 * - Launcher + Intake (toggle Y to turn on/off)
 * - Intake Reverse Toggle (hold X to reverse)
 * - Launcher Unjam (press B for one cycle)
 * - Goal Lock (L Bumper (up), L Trigger (down))
 * - Face (R Bumper (open), R Trigger (close))
 *
 * Created March 19, 2016
 *
 *
 */
public class BlackKnightDemo extends LinearOpMode
{
    DcMotor left_drive;
    DcMotor right_drive;
    DcMotor launcher;
    DcMotor lift;
    DcMotor intake;

    Servo goal;
    Servo face;
    //Motor Powers and Locations
    public static final float GOAL_CLOSED = 0.2f; // bash this
    public static final float GOAL_OPEN = 1f; // bash this
    public static final float FACE_CLOSED = 0.07f; // bash this
    public static final float FACE_OPEN = 0.175f; // bash this
    public static final float INTAKE_FORWARD_POWER = 0.5f; // bash this (I think this means 75% power?)
    public static final float INTAKE_REVERSE_POWER = -0.6f; // bash (can we have negative power to reverse?)
    public static final float LAUNCHER_FORWARD_POWER = 0.90f; // bash
    public static final float LAUNCHER_REVERSE_POWER = -0.75f; // bash

    //Launcher unjam constants
    public static final int UNJAM_TOTAL_TIME = 500; // bash time to unjam motor
    public static final int UNJAM_WAIT = 200; //I believe this is 200ms to cap how long the launcher can be not working
    public static final int MIN_ENCODER_TICKS = 1; // min number of encoder ticks allowed for the launcher cycle to be counted as OK

    // threshold for joystick
    public static final float threshold = 0.1f;

    //Running States
    private static boolean launching = false;
    private static int intaking = 0; // 0=stopped; -1=running reverse; 1=running forward
    private static float launcher_dt = 0; //to keep track of launcher time stopped

    //Constructor
    public BlackKnightDemo(){}

    @Override public void runOpMode() throws InterruptedException {
        left_drive = hardwareMap.dcMotor.get("left_d");
        right_drive = hardwareMap.dcMotor.get("right_d");
        launcher = hardwareMap.dcMotor.get("launcher");
        lift = hardwareMap.dcMotor.get("lift");
        intake = hardwareMap.dcMotor.get("intake");
        goal = hardwareMap.servo.get("goal");
        face = hardwareMap.servo.get("face");

        intake.setDirection(DcMotor.Direction.REVERSE);
        launcher.setDirection(DcMotor.Direction.REVERSE);
        right_drive.setDirection(DcMotor.Direction.REVERSE);
        //launcher.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        waitForStart();

        Button controller1 = new Button();
        Button controller2 = new Button();


        ElapsedTime timer = new ElapsedTime();
        timer.reset(); // to unjam launcher

        float dt;
        float new_time;
        float old_time = 0f;
        float launcherPause = 0f;
        float unjamTime = 0f;

        int dEncoderTicks;
        int newNumEncoderTicks;
        int oldNumEncoderTicks = 0;


        while(true) {

            //============================ Control Map ==========================
            // drive
            float[] drive_stick = new float[]{-gamepad1.left_stick_x, -gamepad1.left_stick_y};

            // launcher
            boolean launching = controller2.toggle(Button.Buttons.Y);
            boolean unjam = controller2.press(Button.Buttons.B);

            // intake
            boolean reverseIntake = controller2.press(Button.Buttons.X);

            // lift
            float lift_power = Range.clip(gamepad2.right_stick_y, -1, 1);

            // goal
            boolean goalOpen = controller2.press(Button.Buttons.LEFT_BUMPER);
            boolean goalClosed = gamepad2.left_trigger>0.5;

            // face
            boolean faceOpen = controller2.press(Button.Buttons.RIGHT_BUMPER);
            boolean faceClosed = gamepad2.right_trigger>0.5;

            //================================Drive===============================
            deadZone(drive_stick);
            float left_power  = drive_stick[1]-drive_stick[0];
            float right_power = drive_stick[1]+drive_stick[0];

            right_power = Range.clip(right_power, -1, 1);
            left_power = Range.clip(left_power, -1, 1);

            right_drive.setPower(right_power*0.6); //scale down 60%
            left_drive.setPower(left_power*0.6);


            //============================Launcher + Intake========================
            // Toggles

            if(reverseIntake) { //if X is pressed, run intake backwards
                intaking = -1;
            }
            else {
                intaking = 0;
            }

            //timer updater for launcher unjam
            new_time = (float) timer.time();
            dt = new_time-old_time;
            old_time = new_time;

            if(launching) {
                //encoder updater for launcher unjam
                newNumEncoderTicks = launcher.getCurrentPosition();
                dEncoderTicks = newNumEncoderTicks - oldNumEncoderTicks;
                oldNumEncoderTicks = newNumEncoderTicks;

                if(intaking==0) {//if intake is not running and we are launching, run intake
                    intaking = 1;
                }

                //
                if(dEncoderTicks < MIN_ENCODER_TICKS && dEncoderTicks >= 0) {
                    launcherPause += dt;
                }
                else {
                    launcherPause = 0;
                }

                if(unjam || launcherPause>=UNJAM_WAIT){
                    launcherPause = 0;
                    unjamTime = UNJAM_TOTAL_TIME; //refills time for unjamming
                }

                if (unjamTime > 0)	{
                    intaking = 0;
                    launcher.setPower(LAUNCHER_REVERSE_POWER);
                    unjamTime -= (dt+75);
                }
                else {
                    launcher.setPower(LAUNCHER_FORWARD_POWER);
                }
            }
            else {
                launcher.setPower(0);
            }

            // Run intake based on intaking state
            if(intaking == 1) {
                intake.setPower(INTAKE_FORWARD_POWER);
            }
            else if(intaking == -1) {
                intake.setPower(INTAKE_REVERSE_POWER);
            }
            else {
                intake.setPower(0);
            }

            //================================Lift================================
            lift.setPower(lift_power);

            //===========================Goal Mechanism==========================
            if(goalOpen) {
                goal.setPosition(GOAL_OPEN);
            }
            else if(goalClosed) {
                goal.setPosition(GOAL_CLOSED);
            }

            //========================Face (Net) Mechanism=======================
            if (faceOpen) {
                face.setPosition(FACE_OPEN);
            }
            else if(faceClosed) {
                face.setPosition(FACE_CLOSED);
            }

            //Refreshes
            try {
                controller1.updateButtons(gamepad1.toByteArray());
                controller2.updateButtons(gamepad2.toByteArray());
            }
            catch (RobotCoreException e) {
                e.printStackTrace();
            }
        }
    }


    //Joystick cleanup functions
    /**
     * Applies scalar multiplication to an array of length two.
     * @param v array to be scaled.
     * @param scalar scalar which will be multiplied with each element of array.
     * @return array after scaling.
     */
    static float[] scale(float[] v, float scalar) {
        v[0] *= scalar;
        v[1] *= scalar;
        return v;
    }


    /**
     * Checks if joystick is within threshold of deadzone. Otherwise scales it (?)
     * @param stick An array for the x and y components of the gamepad joystick.
     */
    static void deadZone(float[] stick) {
        float norm = (float)Math.sqrt(stick[0]*stick[0]+stick[1]*stick[1]);
        if(norm < threshold) {
            stick[0] = 0;
            stick[1] = 0;
        }
        else stick = scale(stick, ((norm-threshold)/(1.0f-threshold))/norm);
    }

    static final float P_0 = 0.0f;
    static final float P_1 = 10/100.0f;
    static final float P_2 = 1.0f;

    static float qBezier(float t) {
        return (t >= 0? 1:-1)*((1-t)*((1-t)*P_0 + t*P_1) + t*((1-t)*P_1 + t*P_2));
    }

}
