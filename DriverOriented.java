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

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

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

@TeleOp(name="DRIVE", group="Linear OpMode")
//@Disabled
public class DriverOriented extends OpMode{

    /* Declare OpMode members. */
    DcMotor RFM, RBM, LFM, LBM, colectMotor, armMotor,LeftLauncher,RightLauncher;
    Servo clawMotor, storageMotor, launchServo;
    VoltageSensor voltSensor;
    WebcamName webcamName = null;

    PIDController           pidDrive, pidRotate;

    Orientation lastAngles = new Orientation();

    double zeroHeading = 0;
    double headingRadians = 0;
    double headingPower = 0;
    double LFMP, LBMP, RFMP, RBMP;

    double correction = 0, globalAngle, rotation;

    static final double XHG = 220;
    static final double YHG = -1070;

    static final double     COUNTS_PER_MOTOR        = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM       = 10.0 ;     // For figuring circumference
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);
    static final double     DRIVE_SPEED     = 0.95;
    static final double     INIT_SPEED      = 0.3;

    static final double CLOSED_POS   = 1.0;     // Maximum rotational position
    static final double OPEN_POS     = 0.5;     // Minimum rotational position
    static final double COLLECT_POS  = 0.0;     // Storage position for collecting rings
    static final double LAUNCH_POS   = 0.42;     // Storage position for launching rings
    double clawPos = OPEN_POS;
    double storagePos = COLLECT_POS;
    double launchPos = 1;
    double armSpeed = 0;
    double colectSpeed = 0;
    double launchSpeed = 0;

    static final double     MAX_VOLTAGE     = 13.56;
    static final double     MIN_VOLTAGE     = 10.24;
    double     MIN_POWER_HG       = 0.9;
    double     MIN_POWER_PS       = 0.82;

    private ElapsedTime runtime = new ElapsedTime();
    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false  ;

    private static final String VUFORIA_KEY =
            "AdNnHg//////AAABmcEoJu2l6k1xiWOXd6UqTFwpk47kNMGjhy2tkkbyMWk06L6v0IDgSxtHshLqSFxMZrOkE+q2gDtQE2UatvXizZeD8eWPhrRPbVaz96Ntsfxghh+R+ym37mtDnjsOuFudLWGigLc+RdxBKlgZpvaieDOl69EnQeOANRBso43K660Wr70zxaczj7VHaXJa+CXyXG0sRB/EPL14NVG4bK4D6d/WNbf0oeLOEg441arADs5dRa9iTMNVSxdeGEjS2AOD1EiURG2bqFC220/4qhWUyoHf0LJnS+UoQgLzEYF/KZpAX3jU/kFWCFrNnKJnLYYOQt4SgA9DSPuctSiQdTYET6xZO9G8bWZcUz+Swfji9aSR";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch        = 25.4f;
    private static final float mmTargetHeight   = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField  = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;

    private boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;
    final double LaunchX          = 266.0;
    final double LaunchY          = -1030.0;
    final double LaunchZ          = 140.5;
    final double LaunchError      = 90.0;

    // Gyro
    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init()
    {
        webcamName = hardwareMap.get(WebcamName.class, "webcam");

        RFM = hardwareMap.dcMotor.get("rfm");
        RBM = hardwareMap.dcMotor.get("rbm");
        LFM = hardwareMap.dcMotor.get("lfm");
        LBM = hardwareMap.dcMotor.get("lbm");
        LeftLauncher=hardwareMap.dcMotor.get("left_launch");
        RightLauncher=hardwareMap.dcMotor.get("right_launch");

        colectMotor = hardwareMap.dcMotor.get("colectMotor");
        storageMotor = hardwareMap.get(Servo.class, "storage");
        launchServo = hardwareMap.get(Servo.class, "launch");
        armMotor = hardwareMap.dcMotor.get("arm");
        clawMotor = hardwareMap.get(Servo.class, "claw");
        voltSensor    = hardwareMap.get(VoltageSensor.class,"Control Hub");

        RBM.setDirection(DcMotor.Direction.REVERSE);
        LBM.setDirection(DcMotor.Direction.REVERSE);
        RFM.setDirection(DcMotor.Direction.REVERSE);
        LFM.setDirection(DcMotor.Direction.REVERSE);

        RFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LFM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        LFM.setPower(0);
        RFM.setPower(0);
        LBM.setPower(0);
        RBM.setPower(0);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile  = "BNo055IMUCalibration.json";
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");

        pidDrive = new PIDController(.05, 0, 0);
        pidRotate = new PIDController(.03, .002, .001);

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, DRIVE_SPEED);
        pidDrive.setInputRange(0, 90);
        pidDrive.setTolerance(0.5);
        pidDrive.enable();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {}

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {}

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop()
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        double front = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotation = gamepad1.right_stick_x;
        launchSpeed=0;
        armSpeed = 0;
        colectSpeed = 0;
        launchPos = 1;

        /// COLLECTOR
        if(gamepad2.x) {
            colectSpeed = 0.95;
            storagePos = COLLECT_POS;
        }
        if(gamepad2.y) {
            colectSpeed = -0.95;
            storagePos  = COLLECT_POS;
        }
        if(gamepad1.right_trigger != 0) {
            colectSpeed = 0.95;
            storagePos  = COLLECT_POS;
        }
        if(gamepad1.left_trigger != 0) {
            colectSpeed = -0.95;
            storagePos  = COLLECT_POS;
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

        ///LAUNCH SERVO
        if(gamepad2.right_trigger != 0)
            launchPos = 0.83;
        if(gamepad1.b)
            launchPos = 0.83;

        if(gamepad1.a)
            storagePos = LAUNCH_POS;

        /// LAUNCH
        if(gamepad2.left_trigger != 0) {
            launchSpeed =  MAX_VOLTAGE * MIN_POWER_HG / voltSensor.getVoltage();
            storagePos = LAUNCH_POS;
        }
        if(gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_up || gamepad2.dpad_right){
            launchSpeed = MAX_VOLTAGE * MIN_POWER_PS / voltSensor.getVoltage();
            storagePos = LAUNCH_POS;
        }

        if(gamepad1.left_bumper && gamepad1.right_bumper)
            zeroHeading = angles.firstAngle;

        double imuHeading = angles.firstAngle - zeroHeading;

        if(front != 0 || right != 0)
        {
            headingRadians = Math.atan2(right, front);
            headingPower = Math.min(Math.sqrt((front * front) + (right * right)), 1);
        }
        else
        {
            headingRadians = 0;
            headingPower = 0;
        }

        LFMP = headingPower * -Math.sin(headingRadians + imuHeading + (Math.PI / 4)) - rotation;
        RFMP = headingPower * Math.cos(headingRadians + imuHeading + (Math.PI / 4)) - rotation;
        RBMP = headingPower * Math.sin(headingRadians + imuHeading + (Math.PI / 4)) - rotation;
        LBMP = headingPower * -Math.cos(headingRadians + imuHeading + (Math.PI / 4)) - rotation;

        double maxRaw = Math.max(Math.max(Math.abs(LFMP), Math.abs(RFMP)), Math.max(Math.abs(RBMP), Math.abs(LBMP)));

        if(maxRaw > 1)
        {
            LFMP /= maxRaw;
            RFMP /= maxRaw;
            RBMP /= maxRaw;
            LBMP /= maxRaw;
        }

        // Send calculated power to wheels
        LFM.setPower(LFMP);
        RBM.setPower(RBMP);
        LBM.setPower(LBMP);
        RFM.setPower(RFMP);

        clawMotor.setPosition(clawPos);
        colectMotor.setPower(colectSpeed);
        storageMotor.setPosition(storagePos);
        launchServo.setPosition(launchPos);
        armMotor.setPower(armSpeed);
        LeftLauncher.setPower(launchSpeed);
        RightLauncher.setPower(-launchSpeed);


        telemetry.addData("Voltage", voltSensor.getVoltage());
        telemetry.addData("Launch Speed", launchSpeed);

        telemetry.addData("Storage Servo Position", storageMotor.getPosition());
        telemetry.addData("Launch Servo Position", launchServo.getPosition());
        telemetry.addData("storagePos", storagePos);
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop()
    {
        LFM.setPower(0);
        RFM.setPower(0);
        LBM.setPower(0);
        RBM.setPower(0);
    }
}
