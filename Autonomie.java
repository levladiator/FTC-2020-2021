package HighFive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomie5;

@Autonomous(name = "Autonomie", group = "Pushbot")
@Disabled
public final class Autonomie extends LinearOpMode{

    private Robot _robot = new Robot();




    @Override
    public void runOpMode() throws InterruptedException {

        _robot.init(hardwareMap);

        waitForStart();

        while(opModeIsActive())
        {
            _robot.driveTrain.driveY(100,1,0);
        }
    }

}
