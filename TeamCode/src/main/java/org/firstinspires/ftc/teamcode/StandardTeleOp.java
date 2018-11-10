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
//@Disabled
public class StandardTeleOp extends OpMode{

    /* Declare OpMode members. */
    HardwareRobot iceRobot = new HardwareRobot(); // use the class created to define a Pushbot's hardware
                                                         // could also use HardwarePushbotMatrix class.

     /* Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        iceRobot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //

        iceRobot.resetEncoder();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double drive;
        double turn;
        double upClimb;
        double downClimb;
        double maxClimbPower = 100 /* percent */ /100.0; // cool formatting ay?
        int maxclimb= 100;
        int minclimb = -3000;
        boolean clawClosed = false;

        int climbpos = iceRobot.climbMotor.getCurrentPosition();

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        drive = -gamepad1.left_stick_y* 0.5;
        turn = gamepad1.right_stick_x* 0.5;
        upClimb = gamepad1.left_trigger;
        downClimb = gamepad1.right_trigger;


        iceRobot.leftDrive.setPower(Range.clip(drive+turn,-1.0,1.0));
        iceRobot.rightDrive.setPower(Range.clip(drive-turn,-1.0,1.0));

        double input = (upClimb-downClimb)*maxClimbPower;
        if (climbpos > maxclimb) {
            iceRobot.climbMotor.setPower(Range.clip(input,-1,0)); // if too low, only lets you go up.
        }
        else if (climbpos < minclimb) {
            iceRobot.climbMotor.setPower(Range.clip(input,0,1)); // if too high, only lets you o down.
        }
        else {
            iceRobot.climbMotor.setPower(input);
        }

        if (gamepad1.x){
            iceRobot.resetEncoder();
        }

        if (gamepad1.b){
            iceRobot.rightClaw.setPosition(iceRobot.SERVO_CENTER);
        }
        if (gamepad1.a) {
            iceRobot.rightClaw.setPosition(iceRobot.SERVO_CLAW_CLOSED);
        }

        if (gamepad1.y) {
            turn = 0;
            drive = 0;

            double extendspeed = gamepad1.right_trigger-gamepad1.left_trigger;

            iceRobot.RightArmExtend.setPower(extendspeed);

            if (gamepad1.dpad_up) {
                iceRobot.rightHand.setPosition();

            }
        }


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
