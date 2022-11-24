package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "TestSensor", group = "Pushbot")
@Disabled
public class TestSensor extends LinearOpMode
{
    final int WhiteCode = -15193303;
    final int Error = 4000000;
    //@Override
    ColorSensor FrontColorSensor;

    public void runOpMode()
    {
        DcMotor LFM,LBM,RFM,RBM;
        LFM = hardwareMap.dcMotor.get("lfm");
        LBM = hardwareMap.dcMotor.get("lbm");
        RFM = hardwareMap.dcMotor.get("rfm");
        RBM = hardwareMap.dcMotor.get("rbm");

        RFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LFM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //FrontColorSensor= hardwareMap.get(ColorSensor.class, "color");
        //FrontColorSensor.enableLed(true);

        waitForStart();

        LFM.setPower(0.75);
        RFM.setPower(-0.75);
        LBM.setPower(0.75);
        RBM.setPower(-0.75);

        sleep(4000);

        LFM.setPower(0);
        RFM.setPower(0);
        LBM.setPower(0);
        RBM.setPower(0);

        /*
        while(FrontColorSensor.argb() < WhiteCode - Error || FrontColorSensor.argb() > WhiteCode+Error)
        {
            telemetry.addData("Culoare", FrontColorSensor.argb());
            telemetry.update();

            LFM.setPower(0.75);
            RFM.setPower(-0.75);
            LBM.setPower(0.75);
            RBM.setPower(-0.75);
        }

        LFM.setPower(0);
        RFM.setPower(0);
        LBM.setPower(0);
        RBM.setPower(0);
         */
        ///WHITE    -15193303(-10000000,-20000000)
    }
}
