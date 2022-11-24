package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Driver Mode 3", group="Linear OpMode")
@Disabled
public class DriverClass3 extends OpMode{

    /* Declare OpMode members. */
    DcMotor RFM, RBM, LFM, LBM, armMotor, RLaunch, LLaunch,colectMotor;
    Servo clawMotor, launchServo;
    VoltageSensor voltSensor;
    static final double CLOSED_POS   = 1.0;     // Maximum rotational position
    static final double OPEN_POS     = 0.5;     // Minimum rotational position
    double clawPos = OPEN_POS;
    double launchPos = 1.0;
    double armSpeed = 0;
    double colectSpeed = 0;
    double launchSpeed = 0;
    static final double     MAX_VOLTAGE   = 13.56;
    static final double     MIN_POWER_HG  = 0.775;  // For High Goal
    static final double     MIN_POWER_PS  = 0.735;  // For Power Shots

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        clawMotor = hardwareMap.get(Servo.class, "claw");
        launchServo = hardwareMap.get(Servo.class, "launch");
        armMotor = hardwareMap.dcMotor.get("arm");
        colectMotor=hardwareMap.dcMotor.get("colectMotor");
        voltSensor = hardwareMap.voltageSensor.get("Control Hub");

        RFM = hardwareMap.dcMotor.get("rfm");
        RBM = hardwareMap.dcMotor.get("rbm");
        LFM = hardwareMap.dcMotor.get("lfm");
        LBM = hardwareMap.dcMotor.get("lbm");

        RLaunch = hardwareMap.dcMotor.get("right_launch");
        LLaunch = hardwareMap.dcMotor.get("left_launch");

        armMotor.setDirection(DcMotor.Direction.REVERSE);
        RLaunch.setDirection(DcMotor.Direction.REVERSE);

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RFM.setDirection(DcMotor.Direction.REVERSE);
        RBM.setDirection(DcMotor.Direction.REVERSE);
        LFM.setDirection(DcMotor.Direction.REVERSE);
        LBM.setDirection(DcMotor.Direction.REVERSE);

        RFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LFM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
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
        // Setup a variable for each drive wheel to save power level for telemetry
        double LFMP, LBMP, RFMP, RBMP;

        double front = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;

        LFMP = front + right + rotation;
        LBMP = front - right + rotation;
        RFMP = - front + right + rotation;
        RBMP = - front - right + rotation;

        // Send calculated power to wheels
        LFM.setPower(LFMP);
        RBM.setPower(RBMP);
        LBM.setPower(LBMP);
        RFM.setPower(RFMP);

        armSpeed = 0;
        colectSpeed = 0;
        launchPos = 0.6;
        launchSpeed = 0;

        /// COLLECTOR
        if(gamepad2.x)
            colectSpeed = 1;
        if(gamepad2.y)
            colectSpeed = -1;
        if(gamepad1.right_trigger != 0) {
            colectSpeed = 1;
        }
        if(gamepad1.left_trigger != 0) {
            colectSpeed = -1;
        }

        /// WOBLLE GOAL DELIVERY
        if(gamepad2.left_bumper)
            armSpeed = -1;
        if(gamepad2.right_bumper)
            armSpeed = 1;

        if(gamepad2.a) {
            clawPos = OPEN_POS;
        }
        if(gamepad2.b) {
            clawPos = CLOSED_POS;
        }

        if(gamepad2.right_trigger != 0)
            launchPos = .1;

        /// LAUNCH
        if(gamepad2.left_trigger != 0) {
            launchSpeed =  MAX_VOLTAGE * MIN_POWER_HG / voltSensor.getVoltage();
        }
        if(gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_up || gamepad2.dpad_right){
            launchSpeed = MAX_VOLTAGE * MIN_POWER_PS / voltSensor.getVoltage();
        }

        clawMotor.setPosition(clawPos);
        launchServo.setPosition(launchPos);
        colectMotor.setPower(colectSpeed);
        armMotor.setPower(armSpeed);
        RLaunch.setPower(launchSpeed);
        LLaunch.setPower(launchSpeed);

        // Send telemetry message to signify robot running;
        telemetry.addData("Navigation", "front (%.2f), right (%.2f), rotation (&.02f)", front, right, rotation);
        telemetry.addData("Servo", "launch (%.2f), claw (%.2f),", launchPos, clawPos);
        telemetry.addData("Motors","Colect (%.2f), Launch(%.2f)", colectSpeed, launchSpeed);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
