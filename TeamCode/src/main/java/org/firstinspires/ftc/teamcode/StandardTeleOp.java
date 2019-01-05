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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Robot: Teleop Drive", group="TeleOp")
public class StandardTeleOp extends OpMode{

    /* Declare OpMode members. */
    HardwareRobot iceRobot = new HardwareRobot(); // creates object of the robot hardware class

    @Override
    public void init() {
        iceRobot.init(hardwareMap); // initialises motors and servos

        telemetry.addData("Say", "Hello Driver");

        iceRobot.resetEncoder(); // resets arm encoder
    }

    @Override
    public void init_loop() { }

    @Override
    public void start() { }

    @Override
    public void loop() {
        double drive;
        double turn;
        double upClimb;
        double downClimb;
        double joyClimb;
        double maxClimbPower = 100 /* percent */ /100.0; // maximum speed of climb motor
        int maxclimb= 100; // maximum value for climb motor encoder
        int minclimb = -3000; // minimum value
        boolean clawClosed = false;
        boolean armMode2 = gamepad1.y; // specifies whether controller is in arm or drive mode

        int climbpos = iceRobot.climbMotor.getCurrentPosition(); // finds current position of climb motor

        drive=0;
        turn=0;
        //drive = (gamepad2.left_stick_y*-.25);
        //turn = (gamepad2.left_stick_x*.25)

        if(!armMode2) {  // if in
            drive += -gamepad1.left_stick_y;
            turn += gamepad1.right_stick_x* 0.5;
        }
        upClimb = gamepad1.left_trigger;
        downClimb = gamepad1.right_trigger;
        joyClimb = gamepad2.right_stick_y;


        if (gamepad1.left_stick_button) {
            drive = drive * 0.25;
        }
        if (gamepad1.right_stick_button) {
            turn = turn * 0.25;
        }


        iceRobot.leftDrive.setPower(Range.clip(drive+turn,-1.0,1.0));
        iceRobot.rightDrive.setPower(Range.clip(drive-turn,-1.0,1.0));

        double input = (upClimb-downClimb+joyClimb)*maxClimbPower;
        if (climbpos > maxclimb) {
            iceRobot.climbMotor.setPower(Range.clip(input,-1,0)); // if too low, only lets you go up.
        }
        else if (climbpos < minclimb) {
            iceRobot.climbMotor.setPower(Range.clip(input,0,1)); // if too high, only lets you o down.
        }
        else {
            iceRobot.climbMotor.setPower(input);
        }

        if (gamepad1.x || gamepad2.x){
            iceRobot.resetEncoder();
        }

        if (gamepad1.b){
            iceRobot.rightClaw.setPosition(iceRobot.SERVO_CENTER);
        }
        if (gamepad1.a) {
            iceRobot.rightClaw.setPosition(iceRobot.SERVO_CLAW_CLOSED);
        }




        double pivotPower = gamepad2.left_stick_y;
        if(armMode2){ pivotPower += -gamepad1.left_stick_y; }
        iceRobot.pivotMotor.setPower(pivotPower);

        double gripStrength = (gamepad2.left_trigger-gamepad2.right_trigger)*2;
        if (armMode2){ gripStrength += gamepad1.right_stick_y; }
        iceRobot.rightHand.setPower(gripStrength);

        if (gamepad2.right_bumper || (armMode2 && gamepad1.left_bumper)) {
            iceRobot.rightGripper.setPosition(.5);
        }
        if (gamepad2.left_bumper || (armMode2 && gamepad1.right_bumper)){
            iceRobot.rightGripper.setPosition(.75);
        }




        telemetry.addData("touch",iceRobot.frontBumper.isPressed());
        telemetry.addData("drive",  "%.2f", drive);
        telemetry.addData("turn", "%.2f", turn);
        //telemetry.addData("turn", "%g", climbpos);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("status","done");
    }
}
