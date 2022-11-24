package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "Accelerare", group = "Pushbot")
@Disabled
public class AccDec extends LinearOpMode {

    /* Declare OpMode members. */
    DistanceSensor sensorRangeR, sensorRangeF, sensorRangeB, sensorRangeL;
    DcMotor LFM,LBM,RFM,RBM, armMotor,leftLauncher,rightLauncher, colectMotor;
    VoltageSensor voltSensor;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR        = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM       = 10.0 ;     // For figuring circumference
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    static final double     DRIVE_SPEED     = 1;
    static final double     INIT_SPEED      = 0.4;
    static final double     TURN_SPEED      = 1;
    static final double     CLOSED_POS      = 1.0;     // Maximum rotational position
    static final double     OPEN_POS        = 0.5;     // Minimum rotational position
    static final double     ARM_SPEED       = 1.0;
    static final double     MAX_VOLTAGE     = 13.56;
     double     MIN_POWER       = 0.81;

    int                     maincase = -1;
    double                  correction = 0, globalAngle, rotation;
    PIDController           pidDrive, pidRotate;


    @Override
    public void runOpMode() {

        LFM = hardwareMap.dcMotor.get("lfm");
        LBM = hardwareMap.dcMotor.get("lbm");
        RFM = hardwareMap.dcMotor.get("rfm");
        RBM = hardwareMap.dcMotor.get("rbm");

        // Initialize IMU(Gyro Sensor)
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Set up PID coefficients
        pidDrive = new PIDController(.05, 0, 0);
        pidRotate = new PIDController(.03, .002, .001);


        // Calibrate IMU(Gyro Sensor)
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(500);
            idle();
        }
        sleep(500);

        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        RFM.setDirection(DcMotor.Direction.REVERSE);
        LFM.setDirection(DcMotor.Direction.REVERSE);
        RBM.setDirection(DcMotor.Direction.REVERSE);
        LBM.setDirection(DcMotor.Direction.REVERSE);

        RFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                LFM.getCurrentPosition(),
                RFM.getCurrentPosition(),
                LBM.getCurrentPosition(),
                RBM.getCurrentPosition());
        telemetry.update();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, DRIVE_SPEED);
        pidDrive.setInputRange(0, 90);
        pidDrive.setTolerance(0.5);
        pidDrive.enable();


        driveY(300, 1, 30, 50, 0);

    }

        public void driveY(double distance, int front, double targetAcc, double targetDec, double angle) {
        double newTargetL;
        double newTargetR;
        double speed = DRIVE_SPEED;
        //int avgLeft = (LFM.getCurrentPosition() + LBM.getCurrentPosition()) / 2;
        //int avgRight = (RFM.getCurrentPosition() + RBM.getCurrentPosition()) / 2;

        RFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double initAngle = pidDrive.getSetpoint();

        pidDrive.setSetpoint(angle);

        if (opModeIsActive()) {
            if(front==1)
            {
                newTargetL = - (double) (distance * COUNTS_PER_CM/Math.sqrt(2.0) );
                newTargetR = + (double) (distance * COUNTS_PER_CM/Math.sqrt(2.0) );
            }
            else {
                newTargetL =  + (double) (distance * COUNTS_PER_CM );
                newTargetR =  - (double) (distance * COUNTS_PER_CM );
            }

            LFM.setTargetPosition((int)(newTargetL));
            //RFM.setTargetPosition((int)(newTargetR));
           // LBM.setTargetPosition((int)(newTargetL));
            //RBM.setTargetPosition((int)(newTargetR));

            // Turn On RUN_TO_POSITION
            LFM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            RBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // reset the timeout time and start motion.
            runtime.reset();

            while (opModeIsActive() &&
                    (LFM.isBusy() )) {
                correction = pidDrive.performPID(getAngle());

                if(front==1)
                {
                   if(RFM.getCurrentPosition() < (distance - targetDec) * COUNTS_PER_CM/Math.sqrt(2))
                        speed = Math.min((DRIVE_SPEED - INIT_SPEED) * Math.abs(RFM.getCurrentPosition())/(targetAcc * COUNTS_PER_CM/Math.sqrt(2)) + INIT_SPEED, 1);
                   else
                   {
                       speed = Math.max(INIT_SPEED + (DRIVE_SPEED - INIT_SPEED) * Math.abs(distance - RFM.getCurrentPosition() / COUNTS_PER_CM * Math.sqrt(2)) / targetDec, INIT_SPEED);
                   }

                    LFM.setPower(-speed + correction);
                    RFM.setPower(speed + correction);
                    LBM.setPower(-speed + correction);
                    RBM.setPower(speed + correction);
                }
                else {
                    LFM.setPower(speed + correction);
                    RFM.setPower(-speed + correction);
                    LBM.setPower(speed + correction);
                    RBM.setPower(-speed + correction);
                }

                telemetry.addData("correction", correction);
                telemetry.addData("distance", RFM.getCurrentPosition() / COUNTS_PER_CM);
                telemetry.addData("right front power", RFM.getPower());
                telemetry.addData("right back power", RBM.getPower());
                telemetry.addData("left front power", LFM.getPower());
                telemetry.addData("left back power", LBM.getPower());
                telemetry.addData("angle", getAngle());
                telemetry.update();
            }

            }

            // Stop all motion;
            LFM.setPower(0);
            RFM.setPower(0);
            LBM.setPower(0);
            RBM.setPower(0);

            // Turn off RUN_TO_POSITION
            LFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // Set old setpoint
            pidDrive.setSetpoint(initAngle);

            sleep(250);
        }

    public void driveX(double distance,String message,int front,double angle) {
        int newTargetF;
        int newTargetB;
        double speed = DRIVE_SPEED;

        double initAngle = pidDrive.getSetpoint();

        pidDrive.setSetpoint(angle);

        RFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (opModeIsActive()) {
            if(front==1)
            {
                newTargetF = -(int) (distance * COUNTS_PER_CM / 1.41);
                newTargetB = (int) (distance * COUNTS_PER_CM / 1.41);
            }
            else {
                newTargetF = (int) (distance * COUNTS_PER_CM / 1.41);
                newTargetB = -(int) (distance * COUNTS_PER_CM / 1.41);
            }

            LFM.setTargetPosition(newTargetF);
            RFM.setTargetPosition(newTargetF);
            //LBM.setTargetPosition(newTargetB);
           // RBM.setTargetPosition(newTargetB);

            // Turn On RUN_TO_POSITION
            LFM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            while (opModeIsActive() &&
                    (LFM.isBusy() && RFM.isBusy()&&LBM.isBusy()&&RBM.isBusy())) {
                correction = pidDrive.performPID(getAngle());

                if(front==1)
                {
                    /*
                    if(RFM.getCurrentPosition() < targetAcc * COUNTS_PER_CM)
                        speed = (DRIVE_SPEED - INIT_SPEED) * RFM.getCurrentPosition()/(targetAcc * COUNTS_PER_CM) + INIT_SPEED;
                    else
                        speed = DRIVE_SPEED;
                    */

                    LFM.setPower(-speed + correction);
                    RFM.setPower(-speed + correction);
                    LBM.setPower(speed + correction);
                    RBM.setPower(speed + correction);

                    telemetry.addData("correction", correction);
                    telemetry.addData("right power", RFM.getPower());
                    telemetry.addData("left power", LFM.getPower());
                    telemetry.addData("angle", getAngle());
                    telemetry.update();
                }
                else {
                    LFM.setPower(speed + correction);
                    RFM.setPower(speed + correction);
                    LBM.setPower(-speed + correction);
                    RBM.setPower(-speed + correction);
                }
            }

        }

        // Stop all motion;
        LFM.setPower(0);
        RFM.setPower(0);
        LBM.setPower(0);
        RBM.setPower(0);

        // Turn off RUN_TO_POSITION
        LFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set old setpoint
        pidDrive.setSetpoint(initAngle);

        sleep(250);
    }

    public void driveFrontUltra(double speed, String message, double fromWall) {
        if (opModeIsActive()) {
            double distance = (double) sensorRangeF.getDistance(DistanceUnit.CM);

            // reset the timeout time and start motion.
            runtime.reset();

            if(distance - fromWall > 0) {
                while (opModeIsActive() && distance > fromWall) {
                    distance = (double) sensorRangeF.getDistance(DistanceUnit.CM);
                    correction = pidDrive.performPID(getAngle());

                    LFM.setPower(-speed + correction);
                    RFM.setPower(speed + correction);
                    LBM.setPower(-speed + correction);
                    RBM.setPower(speed + correction);

                    telemetry.addData("message", message);
                    telemetry.addData("front", distance);
                    telemetry.update();

                }
            }
            else {
                while (opModeIsActive() && distance < fromWall) {
                    distance = (double) sensorRangeF.getDistance(DistanceUnit.CM);
                    correction = pidDrive.performPID(getAngle());

                    LFM.setPower(speed + correction);
                    RFM.setPower(-speed + correction);
                    LBM.setPower(speed + correction);
                    RBM.setPower(-speed + correction);

                    telemetry.addData("message", message);
                    telemetry.addData("front", distance);
                    telemetry.update();

                }
            }
            // Stop all motion;
            LFM.setPower(0);
            RFM.setPower(0);
            LBM.setPower(0);
            RBM.setPower(0);

            // Turn off RUN_TO_POSITION
            LFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }

    public void driveBackUltra(double speed, String message, double fromWall) {
        if (opModeIsActive()) {
            double distance = (double) sensorRangeB.getDistance(DistanceUnit.CM);

            // reset the timeout time and start motion.
            runtime.reset();

            if(distance - fromWall > 0) {
                while (opModeIsActive() && distance > fromWall) {
                    distance = (double) sensorRangeB.getDistance(DistanceUnit.CM);
                    correction = pidDrive.performPID(getAngle());

                    LFM.setPower(speed + correction);
                    RFM.setPower(-speed + correction);
                    LBM.setPower(speed + correction);
                    RBM.setPower(-speed + correction);

                    telemetry.addData("message", message);
                    telemetry.addData("back", distance);
                    telemetry.update();

                }
            }
            else{
                while (opModeIsActive() && distance < fromWall) {
                    distance = (double) sensorRangeB.getDistance(DistanceUnit.CM);
                    correction = pidDrive.performPID(getAngle());

                    LFM.setPower(-speed + correction);
                    RFM.setPower(speed + correction);
                    LBM.setPower(-speed + correction);
                    RBM.setPower(speed + correction);

                    telemetry.addData("message", message);
                    telemetry.addData("back", distance);
                    telemetry.update();
                }
            }


            // Stop all motion;
            LFM.setPower(0);
            RFM.setPower(0);
            LBM.setPower(0);
            RBM.setPower(0);

            // Turn off RUN_TO_POSITION
            LFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }

    public void driveRightUltra(double speed, String message, double fromWall) {
        if (opModeIsActive()) {
            double distance = (double) sensorRangeR.getDistance(DistanceUnit.CM);

            // reset the timeout time and start motion.
            runtime.reset();

            if(distance - fromWall > 0) {
                while (opModeIsActive() && distance > fromWall) {
                    distance = (double) sensorRangeR.getDistance(DistanceUnit.CM);
                    correction = pidDrive.performPID(getAngle());

                    LFM.setPower(-speed + correction);
                    RFM.setPower(-speed + correction);
                    LBM.setPower(speed + correction);
                    RBM.setPower(speed + correction);

                    telemetry.addData("message", message);
                    telemetry.addData("right", distance);
                    telemetry.update();

                }
            }
            else
            {
                while (opModeIsActive() && distance < fromWall) {
                    distance = (double) sensorRangeR.getDistance(DistanceUnit.CM);
                    correction = pidDrive.performPID(getAngle());

                    LFM.setPower(speed + correction);
                    RFM.setPower(speed + correction);
                    LBM.setPower(-speed + correction);
                    RBM.setPower(-speed + correction);

                    telemetry.addData("message", message);
                    telemetry.addData("right", distance);
                    telemetry.update();

                }
            }

            // Stop all motion;
            LFM.setPower(0);
            RFM.setPower(0);
            LBM.setPower(0);
            RBM.setPower(0);

            // Turn off RUN_TO_POSITION
            LFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }

    public void driveLeftUltra(double speed, String message, double fromWall) {
        if (opModeIsActive()) {
            double distance = (double) sensorRangeL.getDistance((DistanceUnit.CM));

            // reset the timeout time and start motion.
            runtime.reset();

            if(distance > fromWall) {
                while (opModeIsActive() && distance > fromWall) {
                    distance = (double) sensorRangeL.getDistance(DistanceUnit.CM);
                    correction = pidDrive.performPID(getAngle());

                    LFM.setPower(speed + correction);
                    RFM.setPower(speed + correction);
                    LBM.setPower(-speed + correction);
                    RBM.setPower(-speed + correction);

                    telemetry.addData("message", message);
                    telemetry.addData("left", distance);
                    telemetry.update();

                }
            }
            else {
                while (opModeIsActive() && distance < fromWall) {
                    distance = (double) sensorRangeL.getDistance(DistanceUnit.CM);
                    correction = pidDrive.performPID(getAngle());

                    LFM.setPower(-speed + correction);
                    RFM.setPower(-speed + correction);
                    LBM.setPower(speed + correction);
                    RBM.setPower(speed + correction);

                    telemetry.addData("message", message);
                    telemetry.addData("left", distance);
                    telemetry.update();

                }
            }
            // Stop all motion;
            LFM.setPower(0);
            RFM.setPower(0);
            LBM.setPower(0);
            RBM.setPower(0);

            // Turn off RUN_TO_POSITION
            LFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            LBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            RBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }

    public void launchRings(int rings)
    {
        double speed = MAX_VOLTAGE * MIN_POWER / voltSensor.getVoltage();
        rightLauncher.setPower(speed);
        leftLauncher.setPower(speed);
        sleep(500);
        telemetry.addData("speed", speed);
        telemetry.addData("voltage", voltSensor.getVoltage());
        telemetry.update();
        rightLauncher.setPower(0);
        leftLauncher.setPower(0);
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
     * Resets the cumulative angle tracking to zero.
     */
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public void rotate(int degrees, double power)
    {
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

        /*
        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, Math.abs(degrees));
        pidRotate.setOutputRange(0, 1);
        pidRotate.setTolerance(1);
        pidRotate.enable();
         */

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
                LFM.setPower(-power);
                RFM.setPower(-power);
                LBM.setPower(-power);
                RBM.setPower(-power);
                sleep(100);

            }
            do
            {
                telemetry.addData("putere",power);
                telemetry.update();
                power = (power - INIT_SPEED) * (degrees - getAngle()) / degrees - INIT_SPEED;// power will be - on right turn.
                LFM.setPower(power);
                RFM.setPower(power);
                LBM.setPower(power);
                RBM.setPower(power);
            } while (opModeIsActive() && getAngle() > degrees);
        }
        else  // left turn.
            do
            {
                telemetry.addData("putere",power);
                telemetry.update();
                power = (power - INIT_SPEED) * (degrees - getAngle()) / degrees + INIT_SPEED; // power will be + on left turn.
                LFM.setPower(power);
                RFM.setPower(power);
                LBM.setPower(power);
                RBM.setPower(power);
            } while (opModeIsActive() && getAngle() < degrees);
        // turn the motors off.
        LFM.setPower(0);
        RFM.setPower(0);
        LBM.setPower(0);
        RBM.setPower(0);
        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);
    }

    public void rotate0(double power)
    {
        pidRotate.reset();
        pidRotate.setSetpoint(0);
        pidRotate.setInputRange(0, 90);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(2);
        pidRotate.enable();

        double degrees = getAngle();

        if(degrees < 0)
        {
            do
            {
                telemetry.addData("putere",power);
                telemetry.update();
                power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                LFM.setPower(power);
                RFM.setPower(power);
                LBM.setPower(power);
                RBM.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        else
        {
            do
            {
                telemetry.addData("putere",power);
                power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                LFM.setPower(power);
                RFM.setPower(power);
                LBM.setPower(power);
                RBM.setPower(power);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }

        // turn the motors off.
        LFM.setPower(0);
        RFM.setPower(0);
        LBM.setPower(0);
        RBM.setPower(0);
        rotation = getAngle();

        // wait for rotation to stop.
        sleep(500);

    }

    public void armUP(boolean close)
    {
        armMotor.setTargetPosition(armMotor.getCurrentPosition() + 2700);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(close == true) {
            sleep(500);
        }

        armMotor.setPower(ARM_SPEED);

        while (opModeIsActive() && armMotor.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder", armMotor.getCurrentPosition() + "   busy =" + armMotor.isBusy());
            telemetry.update();
            idle();
        }
        sleep(150);

        armMotor.setPower(0.0);
    }

    public void armDown(boolean start)
    {
        if(start == true)
            armMotor.setTargetPosition(armMotor.getCurrentPosition() - 3100);
        else
            armMotor.setTargetPosition(armMotor.getCurrentPosition() - 2700);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setPower(-ARM_SPEED);

        while (opModeIsActive() && armMotor.isBusy())   //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
        {
            telemetry.addData("encoder", armMotor.getCurrentPosition() + "   busy =" + armMotor.isBusy());
            telemetry.update();
            idle();
        }
        sleep(150);

        armMotor.setPower(0.0);
    }
}