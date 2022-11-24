// autonomous program that drives bot forward a set distance, stops then
// backs up to the starting point using encoders to measure the distance.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/*  LFM /       \ RFM
 *     /         \
 *
 *     \         /
 *  LBM \       / RBM
 */
@Autonomous(name="Autonomie2", group="Exercises")
@Disabled
public class Autonomie2 extends LinearOpMode
{
    DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;
    //DcMotor armMotor;
    //Servo clawMotor;
    BNO055IMU imu;
    //DistanceSensor distSensor;
    //ColorRangeSensor colorSensor;
    Orientation lastAngles = new Orientation();

    // For CENTIMETERS PER ENCODER COUNT conversion
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM       = 10.0 ;     // For figuring circumference
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    double                  globalAngle, genPower = 1, rotPower = 0.75, correction, rotation;
    int                     maincase;
    PIDController           pidDrive, pidRotate;

    @Override
    public void runOpMode() throws InterruptedException
    {
        //distSensor = hardwareMap.get(DistanceSensor.class, "distance_sensor");
        //colorSensor = hardwareMap.get(ColorRangeSensor.class, "color_sensor");
        //clawMotor = hardwareMap.get(Servo.class, "claw_motor"); // Gheara
        //armMotor=hardwareMap.dcMotor.get("arm_motor"); // Bratul

        leftFrontMotor = hardwareMap.dcMotor.get("lfm");
        leftBackMotor = hardwareMap.dcMotor.get("lbm");
        rightFrontMotor = hardwareMap.dcMotor.get("rfm");
        rightBackMotor = hardwareMap.dcMotor.get("rbm");

        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize IMU(Gyro Sensor)
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Set up PID coefficients
        pidDrive = new PIDController(0.5, 0, 0);
        pidRotate = new PIDController(.003, .00002, .01);


        // Calibrate IMU(Gyro Sensor)
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // wait for start button.
        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        sleep(1000);

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(-45);
        pidDrive.setOutputRange(0, genPower);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        // Case identification
        move(90);
        sleep(1000);
        /*
        double dist1 = colorSensor.getDistance(DistanceUnit.CM); // sus
        double dist2 = distSensor.getDistance(DistanceUnit.CM); // jos
        if(dist1 < 3.0) // 4 rings
        {
            maincase = 4;
        }
        else if (dist2 < 6.0) // 1 ring
        {
            maincase = 1;
        }
        else // 0 rings
        {
            maincase = 0;
        }

        move(135); // Robot next to the middle red square (Zone B)

        // Ramification
        if(maincase == 0)
        {
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rotate(45, rotPower);
            move(-100);
            arm();
            sleep(1000);
            // Arm back w/o servo
            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotor.setTargetPosition(-1440);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            armMotor.setPower(0.7);

            while (opModeIsActive() && armMotor.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
            {
                telemetry.addData("encoder", armMotor.getCurrentPosition() + "   busy =" + armMotor.isBusy());
                telemetry.update();
                idle();
            }
            clawMotor.setPosition(0.0);
            sleep(150);
            armMotor.setPower(0.0);

            move(40);
        }
        else if(maincase == 1)
        {
            arm();
            move(-40);
        }
        else if(maincase == 4)
        {
            leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rotate(-25, rotPower);
            move(80);
            arm();
            move(-120);
        }
        */
    }

    /** Move forward with PID or backwards w/o PID
     * positive = FORWARD; negative = BACKWARDS
     * @param distance (in centimeters)
     */
    public void move(int distance)
    {
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftBackMotor.setTargetPosition((int) (distance * COUNTS_PER_CM));
        rightFrontMotor.setTargetPosition((int) (distance * COUNTS_PER_CM));

        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && leftBackMotor.isBusy())
        {
            telemetry.addData("left", leftBackMotor.getCurrentPosition() + "  busy=" + leftBackMotor.isBusy());
            telemetry.addData("right", rightFrontMotor.getCurrentPosition() + "  busy=" + rightFrontMotor.isBusy());
            //telemetry.addData("dist1", colorSensor.getDistance(DistanceUnit.CM)); // sus
            //telemetry.addData("dist2", distSensor.getDistance(DistanceUnit.CM)); // jos
            //telemetry.addData("case", maincase);
            telemetry.update();

            if(distance > 0) {
                correction = pidDrive.performPID(getAngle());

                leftBackMotor.setPower(genPower - correction);
                rightFrontMotor.setPower(genPower + correction);
            }
            else {
                leftBackMotor.setPower(-genPower);
                rightFrontMotor.setPower(-genPower);
            }
        }
    }

    /*
    // For dropping the Wobble Goal
    public void arm()
    {
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setTargetPosition(1440);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        clawMotor.setPosition(1.0);

        armMotor.setPower(0.7);

        while (opModeIsActive() && armMotor.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder", armMotor.getCurrentPosition() + "   busy =" + armMotor.isBusy());
            telemetry.update();
            idle();
        }
        clawMotor.setPosition(0.0);
        sleep(150);

        armMotor.setPower(0.0);
    }
     */

    /**
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 359 degrees.
     * @param degrees Degrees to turn, + is left - is right
     *                !!! Set RunMode RUN_WITHOUT_ENCODER before using it !!!
     */
    public void rotate(int degrees, double power)
    {
        // restart imu angle tracking.
        resetAngle();

        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);

        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.

        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(0.5);
        pidRotate.enable();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        // rotate until turn is completed.

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                telemetry.addData("putere",power);
                telemetry.update();
                leftFrontMotor.setPower(power);
                rightBackMotor.setPower(-power);
                sleep(100);

            }

            do
            {
                telemetry.addData("putere",power);
                telemetry.update();
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                leftFrontMotor.setPower(-power);
                rightBackMotor.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                telemetry.addData("putere",power);
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                leftFrontMotor.setPower(-power);
                rightBackMotor.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());

        // turn the motors off.
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);

        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

        // reset angle tracking on new heading.
        resetAngle();
    }
}