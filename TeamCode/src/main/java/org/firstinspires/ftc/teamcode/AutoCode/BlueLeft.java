package org.firstinspires.ftc.teamcode.AutoCode;

//-----imports-----
//main imports
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//opencv imports
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

//webcam imports


@Autonomous(name="Blue Left Auto", group="CompetitionAuto")
//@Disabled
public class BlueLeft extends AutoSupplies{
    @Override
    public void runOpMode() {
        int daWay = 0;
        //  Establish all hardware
        initForAutonomous();
        //  Wait until start
        initVision();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        int path = 0;

        telemetry.addData("VisionTime1", path);
        telemetry.update();

        Recognition cone = null;
        long halfSec = 500;
        runtime.reset();
        while(runtime.milliseconds() <= halfSec){
            cone = getConeType();
            if(cone != null){
                break;
            }
        }
        path = getZone(cone);
        telemetry.addLine("Path: " + path);
        telemetry.update();

        sleep(1000);
        if (path == 1) {
            daWay = 1;
        } else if (path == 2) {
            daWay = 2;
        } else if(path == 3){
            daWay = 3;
        } else{ telemetry.addLine("UH OH BOI: " + path);
            telemetry.update(); }


        sleep(300);
        if (daWay == 1) {
            telemetry.addLine("DaWay " + daWay);
            telemetry.update();
            encoderMove(1000, 0.5, 0);
            pause(500);
            encoderMove(1200, 0, 1);
        } else if (daWay == 2) {
            telemetry.addLine("DaWay " + daWay);
            telemetry.update();
            encoderMove(1000, 0.8, 0);
        } else if(daWay == 3){
            telemetry.addLine("DaWay " + daWay);
            telemetry.update();
            encoderMove(1000, 0.5, 0);
            pause(500);
            encoderMove(1200, 0, 1);
        } else{ telemetry.addLine("UH OH BOI: " + path);
            telemetry.update(); }
        //clawOpen();
        telemetry.addLine("DOOOOONNNEEEE " + daWay);
        telemetry.update();
    }
}
//Possible link to add voltage sensor into our code.
//https://www.reddit.com/r/FTC/comments/5cnilm/help_how_to_get_robot_battery_levelvoltage/