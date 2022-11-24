package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;


@Autonomous(name = "Autonomie4", group = "Pushbot")
@Disabled
public class Autonomie4 extends LinearOpMode {

    /* Declare OpMode members. */
    DcMotor LFM,LBM,RFM,RBM;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_CM       = 10.0 ;     // For figuring circumference
    static final double     COUNTS_PER_CM           = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);

    static final double DRIVE_SPEED = 1;
    static final double TURN_SPEED = 0.5;

    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    int maincase = -1;
    double correction, globalAngle;
    PIDController           pidDrive, pidRotate;

    private static final String VUFORIA_KEY =
            "AdNnHg//////AAABmcEoJu2l6k1xiWOXd6UqTFwpk47kNMGjhy2tkkbyMWk06L6v0IDgSxtHshLqSFxMZrOkE+q2gDtQE2UatvXizZeD8eWPhrRPbVaz96Ntsfxghh+R+ym37mtDnjsOuFudLWGigLc+RdxBKlgZpvaieDOl69EnQeOANRBso43K660Wr70zxaczj7VHaXJa+CXyXG0sRB/EPL14NVG4bK4D6d/WNbf0oeLOEg441arADs5dRa9iTMNVSxdeGEjS2AOD1EiURG2bqFC220/4qhWUyoHf0LJnS+UoQgLzEYF/KZpAX3jU/kFWCFrNnKJnLYYOQt4SgA9DSPuctSiQdTYET6xZO9G8bWZcUz+Swfji9aSR";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
       // robot.init(hardwareMap);
        LFM = hardwareMap.dcMotor.get("lfm");
        LBM = hardwareMap.dcMotor.get("lbm");
        RFM = hardwareMap.dcMotor.get("rfm");
        RBM= hardwareMap.dcMotor.get("rbm");

        // Initialize IMU(Gyro Sensor)
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        // Set up PID coefficients
        pidDrive = new PIDController(.08, .00002, .01);
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

        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        /**
        RFM.setDirection(DcMotor.Direction.REVERSE);
        LFM.setDirection(DcMotor.Direction.REVERSE);
        RBM.setDirection(DcMotor.Direction.REVERSE);
        LBM.setDirection(DcMotor.Direction.REVERSE);
        **/

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

        // Case Identification
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() == 0 ) {
                        // empty list.  no objects recognized.
                        telemetry.addData("TFOD", "No items detected.");
                        telemetry.addData("Target Zone", "A");
                        maincase = 0;
                    } else {
                        // list is not empty.
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            // check label to see which target zone to go after.
                            if (recognition.getLabel().equals("Single")) {
                                telemetry.addData("Target Zone", "B");
                                maincase = 1;
                            } else if (recognition.getLabel().equals("Quad")) {
                                telemetry.addData("Target Zone", "C");
                                maincase = 4;
                            } else {
                                telemetry.addData("Target Zone", "UNKNOWN");
                            }
                        }
                    }

                    telemetry.update();
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        // Driving

        // Set up parameters for driving in a straight line.
        pidDrive.setSetpoint(-90);
        pidDrive.setOutputRange(0, DRIVE_SPEED);
        pidDrive.setInputRange(-90, 90);
        pidDrive.enable();

        //TO TURN LEFT, USE - INCHES
        //TO TURN RIGHT, USE + INCHES
        sleep(20);
        drive(DRIVE_SPEED,90,5,"Driving to line");

        sleep(1000);     // pause for servos to move

        telemetry.addData("case ", maincase);
        telemetry.update();

        sleep(5000);
    }

    public void drive(double speed, double distance, double timeoutS,String message) {
        int newTargetL;
        int newTargetR;
        int avgLeft = (LFM.getCurrentPosition() + LBM.getCurrentPosition()) / 2;
        int avgRight = (RFM.getCurrentPosition() + RBM.getCurrentPosition()) / 2;
        if (opModeIsActive()) {
            newTargetL = avgLeft - (int) (distance * COUNTS_PER_CM);
            newTargetR = avgRight + (int) (distance * COUNTS_PER_CM);
            LFM.setTargetPosition(newTargetL);
            RFM.setTargetPosition(newTargetR);
            LBM.setTargetPosition(newTargetL);
            RBM.setTargetPosition(newTargetR);


            // Turn On RUN_TO_POSITION
            LFM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LFM.isBusy() && RFM.isBusy()&&LBM.isBusy()&&RBM.isBusy())) {
                correction = pidDrive.performPID(getAngle());

                LFM.setPower((speed - correction));
                RFM.setPower((speed + correction));
                LBM.setPower((speed - correction));
                RBM.setPower((speed + correction));

                telemetry.addData(message,timeoutS);
                telemetry.addData("Front Left ",LFM.getCurrentPosition());
                telemetry.addData("TARGET:",LFM.getTargetPosition());
                telemetry.addData("Front Right ",RFM.getCurrentPosition());
                telemetry.addData("TARGET:",RFM.getTargetPosition());
                telemetry.addData("Back Left ",LBM.getCurrentPosition());
                telemetry.addData("TARGET:",LBM.getTargetPosition());
                telemetry.addData("Back Right ",RBM.getCurrentPosition());
                telemetry.addData("TARGET:",RBM.getTargetPosition());
                telemetry.addData("CASE",maincase);

                telemetry.update();

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

    public void turn(double speed, double distance, double timeoutS,String message) {
        int newTargetL;
        int newTargetR;
        int avgLeft = (LFM.getCurrentPosition() + LBM.getCurrentPosition()) / 2;
        int avgRight = (RFM.getCurrentPosition() + RBM.getCurrentPosition()) / 2;
        if (opModeIsActive()) {
            newTargetL = avgLeft + (int) (distance * COUNTS_PER_CM);
            newTargetR = avgRight + (int) (distance * COUNTS_PER_CM);
            LFM.setTargetPosition(newTargetL);
            RFM.setTargetPosition(newTargetR);
            LBM.setTargetPosition(newTargetL);
            RBM.setTargetPosition(newTargetR);
            // Turn On RUN_TO_POSITION
            LFM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RFM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LBM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RBM.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            LFM.setPower(Math.abs(speed));
            RFM.setPower(Math.abs(speed));
            LBM.setPower(Math.abs(speed));
            RBM.setPower(Math.abs(speed));

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (LFM.isBusy() && RFM.isBusy())) {
                telemetry.addData(message,timeoutS);
                telemetry.addData("Front Left ",LFM.getCurrentPosition());
                telemetry.addData("TARGET:",LFM.getTargetPosition());
                telemetry.addData("Front Right ",RFM.getCurrentPosition());
                telemetry.addData("TARGET:",RFM.getTargetPosition());
                telemetry.addData("Back Left ",LBM.getCurrentPosition());
                telemetry.addData("TARGET:",LBM.getTargetPosition());
                telemetry.addData("Back Right ",RBM.getCurrentPosition());
                telemetry.addData("TARGET:",RBM.getTargetPosition());


                telemetry.update();

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
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.7f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}


