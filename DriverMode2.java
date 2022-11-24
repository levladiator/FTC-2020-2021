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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="DriverMode2", group="Linear Opmode")
@Disabled
public class DriverMode2 extends LinearOpMode {

    DcMotor leftMotor, rightMotor;
    float   leftPower, rightPower, xValue, yValue;
    DcMotor armMotor;
    Servo clawMotor;
    boolean rbButton, lbButton,xButton,bButton;

    // called when init button is pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");

        armMotor  = hardwareMap.get(DcMotor.class, "arm_motor");
        clawMotor = hardwareMap.get(Servo.class, "claw_motor");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        armMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        while (opModeIsActive())
        {
            yValue = gamepad1.left_stick_y * -1;
            xValue = gamepad1.left_stick_x * -1;

            leftPower =  yValue - xValue;
            rightPower = yValue + xValue;

            leftMotor.setPower(Range.clip(leftPower, -1.0, 1.0));
            rightMotor.setPower(Range.clip(rightPower, -1.0, 1.0));

            rbButton=gamepad2.right_bumper;
            lbButton=gamepad2.left_bumper;
            xButton=gamepad2.x;
            bButton=gamepad2.b;

            if(rbButton)arm_go();
            if(lbButton)arm_back();
            if(xButton)claw_close();
            if(bButton)claw_open();

            telemetry.addData("Mode", "running");
            telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
            telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
            telemetry.addData("rbButton", rbButton);
            telemetry.addData("lbButton", lbButton);
            telemetry.addData("xButton", xButton);
            telemetry.addData("bButton", bButton);
            telemetry.update();

            idle();
        }
    }
    public void arm_go()
    {
        while(opModeIsActive() && rbButton)
        {
            rbButton = gamepad2.right_bumper;
            armMotor.setPower(0.6);
        }
        armMotor.setPower(0.0);
    }
    public void arm_back()
    {
        while(opModeIsActive() && lbButton)
        {
            lbButton = gamepad2.left_bumper;
            armMotor.setPower(-0.6);
        }
        armMotor.setPower(0.0);
    }
    public void claw_close()
    {
        clawMotor.setPosition(1.0);
    }
    public void claw_open()
    {
        clawMotor.setPosition(0.0);
    }

}
