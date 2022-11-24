package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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


@Autonomous(name = "Start Case", group = "Pushbot")
@Disabled
public class ChooseStart extends LinearOpMode
{
    DistanceSensor sensorRangeR, sensorRangeF, sensorRangeB, sensorRangeL;
    DcMotor LFM, LBM, RFM, RBM, armMotor, leftLauncher, rightLauncher, colectMotor;
    Servo clawMotor, launchServo;
    VoltageSensor voltSensor;
    ColorSensor FrontColorSensor;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();

    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR        = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM       = 10.0 ;     // For figuring circumference
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    static final double     DRIVE_SPEED     = 0.95;
    static final double     INIT_SPEED      = 0.3;
    static final double     TURN_SPEED      = 1;
    static final double     CLOSED_POS      = 1.0;     // Maximum rotational position
    static final double     OPEN_POS        = 0.5;     // Minimum rotational position
    static final double     ARM_SPEED       = 1.0;
    static final double     MAX_VOLTAGE     = 13.56;
    double     MIN_POWER       = 0.81;
    static final int WhiteCode = -15193303;
    static final int ColorError = 4000000;

    static int              maincase = -1;
    static int              line = -1;
    double                  correction = 0, globalAngle, rotation;
    PIDController           pidDrive, pidRotate;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private static final String VUFORIA_KEY = "AdNnHg//////AAABmcEoJu2l6k1xiWOXd6UqTFwpk47kNMGjhy2tkkbyMWk06L6v0IDgSxtHshLqSFxMZrOkE+q2gDtQE2UatvXizZeD8eWPhrRPbVaz96Ntsfxghh+R+ym37mtDnjsOuFudLWGigLc+RdxBKlgZpvaieDOl69EnQeOANRBso43K660Wr70zxaczj7VHaXJa+CXyXG0sRB/EPL14NVG4bK4D6d/WNbf0oeLOEg441arADs5dRa9iTMNVSxdeGEjS2AOD1EiURG2bqFC220/4qhWUyoHf0LJnS+UoQgLzEYF/KZpAX3jU/kFWCFrNnKJnLYYOQt4SgA9DSPuctSiQdTYET6xZO9G8bWZcUz+Swfji9aSR";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode()
    {
        initAll();

       while(1==1)
       {
           while(line == -1)
           {    telemetry.addData("Choose Starting Position: ", "X  for Blue   or  B for Red");
               telemetry.addData("                          ", "LB for Left  or  RB for Right");
               telemetry.update();
               if(gamepad1.x && gamepad1.left_bumper)
                   line = 1;
               else if(gamepad1.x && gamepad1.right_bumper)
                   line = 2;
               else if(gamepad1.b && gamepad1.left_bumper)
                   line = 3;
               else if(gamepad1.b && gamepad1.right_bumper)
                   line = 4;
           }
           telemetry.addData("Are you sure?", "  Press Start to confirm   or   Back to choose another location");
           telemetry.addData("Line",line);
           telemetry.update();

           if(gamepad1.start)
           {
               break;
           }
           else
           {
               if(gamepad1.back)
               {
                   line=-1;
               }
           }

       }
       telemetry.addData("Selected Line",line);
       telemetry.addData("Ready for Start","OK");
       telemetry.update();

        waitForStart();

        telemetry.addData("Line", line);
        telemetry.update();

        //CaseID();

        //telemetry.addData("Case: ", maincase);
        //telemetry.update();
    }

    public void driveY(double distance, int front, double targetAcc, double targetDec, double angle)
    {
        double newTargetL;
        double newTargetR;
        double speed = DRIVE_SPEED;
        double initAngle = pidDrive.getSetpoint();

        RFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pidDrive.setSetpoint(angle);

        if(front==1)
        {
            newTargetL = - (double) (distance * COUNTS_PER_CM/Math.sqrt(2.0) );
            newTargetR = + (double) (distance * COUNTS_PER_CM/Math.sqrt(2.0) );
        }
        else
        {
            newTargetL = + (double) (distance * COUNTS_PER_CM/Math.sqrt(2.0) );
            newTargetR = - (double) (distance * COUNTS_PER_CM/Math.sqrt(2.0) );
        }

        LFM.setTargetPosition((int)(newTargetL));

        LFM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RFM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        runtime.reset();

        while (LFM.isBusy())
        {
            correction = pidDrive.performPID(getAngle());

            if(front==1)
            {
                if(Math.abs(LFM.getCurrentPosition()) < (distance - targetDec) * COUNTS_PER_CM/Math.sqrt(2))
                {
                    speed = Math.min((DRIVE_SPEED - INIT_SPEED) * Math.abs(LFM.getCurrentPosition())/(targetAcc * COUNTS_PER_CM/Math.sqrt(2)) + INIT_SPEED, 1);
                }
                else
                {
                    speed = Math.max(INIT_SPEED + (DRIVE_SPEED - INIT_SPEED) * Math.abs(distance - LFM.getCurrentPosition() / COUNTS_PER_CM * Math.sqrt(2)) / targetDec, INIT_SPEED);
                }

                LFM.setPower(-speed + correction);
                RFM.setPower(speed + correction);
                LBM.setPower(-speed + correction);
                RBM.setPower(speed + correction);

                telemetry.addData("Correction: ", correction);
                telemetry.addData("Speed: ", speed);
                telemetry.addData("LFM: ",LFM.getPower());
                telemetry.addData("RFM: ",RFM.getPower());
                telemetry.addData("LBM: ",LBM.getPower());
                telemetry.addData("RBM: ",RBM.getPower());
                telemetry.update();
            }
            else
            {
                if(Math.abs(LFM.getCurrentPosition()) < (distance - targetDec) * COUNTS_PER_CM/Math.sqrt(2))
                {
                    speed = Math.min((DRIVE_SPEED - INIT_SPEED) * Math.abs(LFM.getCurrentPosition())/(targetAcc * COUNTS_PER_CM/Math.sqrt(2)) + INIT_SPEED, 1);
                }
                else
                {
                    speed = Math.max(INIT_SPEED + (DRIVE_SPEED - INIT_SPEED) * Math.abs(distance - LFM.getCurrentPosition() / COUNTS_PER_CM * Math.sqrt(2)) / targetDec, INIT_SPEED);
                }

                LFM.setPower(speed + correction);
                RFM.setPower(-speed + correction);
                LBM.setPower(speed + correction);
                RBM.setPower(-speed + correction);
            }
        }

        LFM.setPower(0);
        RFM.setPower(0);
        LBM.setPower(0);
        RBM.setPower(0);

        LFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidDrive.setSetpoint(initAngle);

    }

    public void driveX(double distance, int right, double targetAcc, double targetDec, double angle)
    {
        double newTargetF;
        double newTargetB;
        double speed = DRIVE_SPEED;
        double initAngle = pidDrive.getSetpoint();

        RFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LFM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        pidDrive.setSetpoint(angle);

        if(right==1)
        {
            newTargetF = - (double) (distance * COUNTS_PER_CM/Math.sqrt(2.0) );
            newTargetB = + (double) (distance * COUNTS_PER_CM/Math.sqrt(2.0) );
        }
        else
        {
            newTargetF = + (double) (distance * COUNTS_PER_CM/Math.sqrt(2.0) );
            newTargetB = - (double) (distance * COUNTS_PER_CM/Math.sqrt(2.0) );
        }

        RFM.setTargetPosition((int)(newTargetF));

        LFM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RFM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RBM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        runtime.reset();

        while (RFM.isBusy())
        {
            correction = pidDrive.performPID(getAngle());

            if(right==1)
            {
                if(Math.abs(RFM.getCurrentPosition()) < (distance - targetDec) * COUNTS_PER_CM/Math.sqrt(2))
                {
                    speed = Math.min((DRIVE_SPEED - INIT_SPEED) * Math.abs(RFM.getCurrentPosition())/(targetAcc * COUNTS_PER_CM/Math.sqrt(2)) + INIT_SPEED, 1);
                }
                else
                {
                    speed = Math.max(INIT_SPEED + (DRIVE_SPEED - INIT_SPEED) * Math.abs(distance - RFM.getCurrentPosition() / COUNTS_PER_CM * Math.sqrt(2)) / targetDec, INIT_SPEED);
                }

                LFM.setPower(-speed + correction);
                RFM.setPower(-speed + correction);
                LBM.setPower(speed + correction);
                RBM.setPower(speed + correction);

                telemetry.addData("Correction: ", correction);
                telemetry.addData("Speed: ", speed);
                telemetry.addData("LFM: ",LFM.getPower());
                telemetry.addData("RFM: ",RFM.getPower());
                telemetry.addData("LBM: ",LBM.getPower());
                telemetry.addData("RBM: ",RBM.getPower());
                telemetry.update();
            }
            else
            {
                if(Math.abs(RFM.getCurrentPosition()) < (distance - targetDec) * COUNTS_PER_CM/Math.sqrt(2))
                {
                    speed = Math.min((DRIVE_SPEED - INIT_SPEED) * Math.abs(RFM.getCurrentPosition())/(targetAcc * COUNTS_PER_CM/Math.sqrt(2)) + INIT_SPEED, 1);
                }
                else
                {
                    speed = Math.max(INIT_SPEED + (DRIVE_SPEED - INIT_SPEED) * Math.abs(distance - RFM.getCurrentPosition() / COUNTS_PER_CM * Math.sqrt(2)) / targetDec, INIT_SPEED);
                }

                LFM.setPower(speed + correction);
                RFM.setPower(speed + correction);
                LBM.setPower(-speed + correction);
                RBM.setPower(-speed + correction);
            }
        }

        LFM.setPower(0);
        RFM.setPower(0);
        LBM.setPower(0);
        RBM.setPower(0);

        LFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RFM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RBM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pidDrive.setSetpoint(initAngle);

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
        for(int i=0;i<rings;++i)
        {
            launchServo.setPosition(0.1);
            sleep(700);
            launchServo.setPosition(0.6);
            sleep(500);
        }
        rightLauncher.setPower(0);
        leftLauncher.setPower(0);
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right from zero point.
     */
    private double getAngle()
    {
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
            clawMotor.setPosition(CLOSED_POS);
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

        clawMotor.setPosition(OPEN_POS);

        armMotor.setPower(0.0);
    }

    private void initVuforia()
    {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTfod()
    {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);

        if (tfod != null)
        {
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
        }
    }

    private void initHardware()
    {
        LFM           = hardwareMap.get(DcMotor.class, "lfm");
        LBM           = hardwareMap.get(DcMotor.class, "lbm");
        RFM           = hardwareMap.get(DcMotor.class, "rfm");
        RBM           = hardwareMap.get(DcMotor.class, "rbm");

        //clawMotor     = hardwareMap.get(Servo.class, "claw");
        //launchServo   = hardwareMap.get(Servo.class, "launch");
        //armMotor      = hardwareMap.get(DcMotor.class, "arm");
        //colectMotor   = hardwareMap.get(DcMotor.class, "colectMotor");
        //leftLauncher  = hardwareMap.get(DcMotor.class, "left_launch");
        //rightLauncher = hardwareMap.get(DcMotor.class, "right_launch");
        voltSensor    = hardwareMap.get(VoltageSensor.class,"Control Hub");

        imu           = hardwareMap.get(BNO055IMU.class, "imu");

        FrontColorSensor = hardwareMap.get(ColorSensor.class, "color");
    }

    private void initIMU()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled       = false;
        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(500);
            idle();
        }
        sleep(500);
    }

    private void initAux()
    {
        //armMotor.setDirection(DcMotor.Direction.REVERSE);
        //rightLauncher.setDirection(DcMotor.Direction.REVERSE);
        //armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        //clawMotor.setPosition(CLOSED_POS);
        //launchServo.setPosition(1);

        FrontColorSensor.enableLed(true);
    }

    private void initPID()
    {
        pidDrive = new PIDController(.05, 0, 0);
        pidRotate = new PIDController(.03, .002, .001);

        pidDrive.setSetpoint(0);
        pidDrive.setOutputRange(0, DRIVE_SPEED);
        pidDrive.setInputRange(0, 90);
        pidDrive.setTolerance(0.5);
        pidDrive.enable();
    }

    private void initAll()
    {
        initHardware();
        initIMU();
        initVuforia();
        initTfod();
        initAux();
        initPID();
    }

    private void CaseID()
    {
        if (tfod != null)
        {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null)
            {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 0 )
                {
                    maincase = 0;
                }
                else
                {
                    int i = 0;
                    for (Recognition recognition : updatedRecognitions)
                    {
                        if (recognition.getLabel().equals("Single"))
                        {
                            maincase = 1;
                        }
                        else if (recognition.getLabel().equals("Quad"))
                        {
                            maincase = 4;
                        }
                    }
                }
                telemetry.update();
            }
        }
        if (tfod != null)
        {
            tfod.shutdown();
        }
    }
}