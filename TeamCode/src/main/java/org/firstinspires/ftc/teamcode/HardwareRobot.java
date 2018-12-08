/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Robot init routine:
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "climb_motor"
 */

public class HardwareRobot // TODO (andrew): doesn't really matter but maybe rename this IceRobot / HardwareIceBot? It is really a personal preference.
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;  //initalizes the drive motors
    public DcMotor  rightDrive  = null;
    public DcMotor  climbMotor  = null;  //initalizes the climb motor
    public DcMotor  pivotMotor  = null;


    public Servo rightClaw   = null;  // initializes servo claw
    public Servo rightGripper = null; // initializes servo gripper
    public CRServo rightHand  = null;   // init


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime time  = new ElapsedTime();

    /* variables*/

    public boolean doneflag = false;

    public static final int maxpos = -3000;
    public static final int minpos = 0;

    public static final double SERVO_CENTER = 0.5;
    public static final double SERVO_CLAW_CLOSED = 1;
    public static final double SERVO_CLAW_OPEN = 0;


    public static final double turn_diameter = 15.4; // inches
    public static final double wheel_diameter = 4;   // inches
    public static final double encoder_ticks_per_revolution = 1120;
    public static final double ticks_per_degree =
            (turn_diameter/wheel_diameter)*
            (encoder_ticks_per_revolution/360);
    public static final double encoder_ticks_per_inch =
            encoder_ticks_per_revolution/(wheel_diameter * Math.PI);

    /* Constructor */
    public HardwareRobot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        climbMotor = hwMap.get(DcMotor.class, "climb_motor");
        pivotMotor = hwMap.get(DcMotor.class,"right_pivot_motor");


        //define and initialize Servos
        rightClaw = hwMap.get(Servo.class, "right_claw");
        rightGripper = hwMap.get(Servo.class,"right_gripper");
        rightHand = hwMap.get(CRServo.class,"right_hand");

        rightClaw.setPosition(SERVO_CLAW_CLOSED);

        rightGripper.setPosition(.75);
        rightHand.setPower(0);

        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        climbMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        climbMotor.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetEncoder () {
        climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void resetEncoders () {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveTime(double speed, double turn){
        rightDrive.setPower(speed+turn);
        leftDrive.setPower(speed-turn);
    }
    public void stop() {
        rightDrive.setPower(0);
        leftDrive.setPower(0);
    }
    public void encoderTurn(double degrees){

        int ticks = (int) (ticks_per_degree * degrees);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (degrees > 0){
            rightDrive.setPower(-.25);
            leftDrive.setPower(.25);
        }else {
            rightDrive.setPower(.25);
            leftDrive.setPower(-.25);
        }
        boolean done = false;
        while(!done) {if (Math.abs(leftDrive.getCurrentPosition()) > Math.abs(ticks)
                || doneflag){
            done = true;
        }
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    public void encoderMove(double inches, double speed, LinearOpMode opMode){
        int ticks = (int) (encoder_ticks_per_inch * inches);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (inches > 0){
            rightDrive.setPower(speed);
            leftDrive.setPower(speed);
        }else {
            rightDrive.setPower(-speed);
            leftDrive.setPower(-speed);
        }
        boolean done = false;
        while(!done){
            if( Math.abs(leftDrive.getCurrentPosition())>Math.abs(ticks)
                    || doneflag
                    || !opMode.opModeIsActive()){
                done = true;
            }
        }
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
 }

