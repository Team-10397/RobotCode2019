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
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Robot init routine:
 * Refer to hardwaremap.txt
 */

public class HardwareRobot // this is the shared class between all of our opmodes
{

    // Hello! 
    
    /* Public OpMode members. */
    // Define Motors
    public DcMotor  leftDrive   = null;  //initializes the drive motors
    public DcMotor  rightDrive  = null;
    public DcMotor  climbMotor  = null;  //initializes the climb motor
    public DcMotor  pivotMotor  = null;  //initializes the arm motor

    // Define Servos
    public Servo rightClaw   = null;  // initializes servo claw
    public Servo rightGripper = null; // initializes servo gripper as servo
    //public CRServo rightGripper = null; // initializes servo gripper as continuous servo
    public CRServo rightHand  = null; // initializes arm wrist servo

    // Define Sensors
    public TouchSensor frontBumper = null; //defines touch sensor


    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime time  = new ElapsedTime();

    /* variables*/

    // these are the extremities of the climb motor
    public static final int maxpos = -3000;
    public static final int minpos = 0;

    // constants for servo position
    public static final double SERVO_CENTER = 0.5;
    public static final double SERVO_CLAW_CLOSED = 1;
    public static final double SERVO_CLAW_OPEN = 0;

    // constants for encoder moves
    public static final double turn_diameter = 15.4; // inches
    public static final double wheel_diameter = 4;   // inches
    public static final double encoder_ticks_per_revolution = 1120;
    public static final double ticks_per_degree =
            (turn_diameter/wheel_diameter)*
            (encoder_ticks_per_revolution/360); //used in encoder turn
    public static final double encoder_ticks_per_inch =
            encoder_ticks_per_revolution/
                    (wheel_diameter * Math.PI); // used in encoder drive

    /* Constructor */
    public HardwareRobot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        // Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        climbMotor = hwMap.get(DcMotor.class, "climb_motor");
        pivotMotor = hwMap.get(DcMotor.class,"right_pivot_motor");


        // Initialize Servos
        rightClaw = hwMap.get(Servo.class, "right_claw");
        rightGripper = hwMap.get(Servo.class,"right_gripper");
        //rightGripper = hwMap.get(CRServo.class,"right_gripper");
        rightHand = hwMap.get(CRServo.class,"right_hand");

        // Initialize Sensors
        frontBumper = hwMap.get(TouchSensor.class,"front_bumper");

        // Move servos to default positions
        rightClaw.setPosition(SERVO_CLAW_CLOSED);

        rightGripper.setPosition(.75);
        //rightGripper.setPower(0);
        rightHand.setPower(0);

        // set motor directions
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        climbMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        climbMotor.setPower(0);


        // Set all motors to run with encoders.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    //the following function resets the climb motor encoder
    public void resetEncoder () {
        climbMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        climbMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // the following function resets the drive motor encoders
    public void resetEncoders () {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // the following function sets the drivetrain to a specific speed
    public void moveTime(double speed, double turn){
        rightDrive.setPower(speed+turn);
        leftDrive.setPower(speed-turn);
    }

    // the following function stops the robot
    public void stop() {
        rightDrive.setPower(0);
        leftDrive.setPower(0);
    }

    // the following function turns to the specified angle using encoders
    public void encoderTurn(double degrees){
        // calculates encoder ticks to turn motors
        int ticks = (int) (ticks_per_degree * degrees);
        resetEncoders();
        // turns left or right, depending on the input
        if (degrees > 0){
            rightDrive.setPower(-.25);
            leftDrive.setPower(.25);
        }else {
            rightDrive.setPower(.25);
            leftDrive.setPower(-.25);
        }
        // waits until the motors have turned to the specified angle
        boolean done = false;
        while(!done) {
            if (Math.abs(leftDrive.getCurrentPosition()) > Math.abs(ticks)){
                done = true;
            }
        }
        stop();
    }

    // the following function moves the specified number of inches
    public void encoderMove(double inches, double speed, LinearOpMode opMode){
        //calculates distance to move motors
        int ticks = (int) (encoder_ticks_per_inch * inches);
        resetEncoders();
        // go forward or backward, depending on input
        if (inches > 0){
            rightDrive.setPower(speed);
            leftDrive.setPower(speed);
        }else {
            rightDrive.setPower(-speed);
            leftDrive.setPower(-speed);
        }
        // waits until motors  have traveled the specified distance
        boolean done = false;
        while(!done){
            if( Math.abs(leftDrive.getCurrentPosition())>Math.abs(ticks)
                    || !opMode.opModeIsActive()){
                done = true;
            }
        }
        stop();
    }
 }

