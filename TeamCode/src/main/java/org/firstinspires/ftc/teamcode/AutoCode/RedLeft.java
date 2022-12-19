package org.firstinspires.ftc.teamcode.AutoCode;

//-----imports-----
//main imports
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//opencv imports
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.AutoCode.AutoSupplies;

//webcam imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@Autonomous(name="RedLeft Auto", group="CompetitionAuto")
//@Disabled
public class RedLeft extends AutoSupplies {
    @Override
    public void runOpMode() {
        int daWay = 0;
        //  Establish all hardware
        initForAutonomous();
        //  Wait until start
        //waitForStart();
        initVision();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        int path = 0;

        Recognition cone = null;
        long halfSec = 700;
        runtime.reset();
        while (runtime.milliseconds() <= halfSec) {
            cone = getConeType();
            if (cone != null) {
                break;
            }
        }
        path = getZone(cone);
        telemetry.addLine("Path: " + path);
        telemetry.update();

        sleep(300);
        if (path == 1) {
            daWay = 1;
        } else if (path == 2) {
            daWay = 2;
        } else if(path == 3){
            daWay = 3;
        } else{ telemetry.addLine("UH OH BOI: " + path);
            telemetry.update(); }

        sleep(300);
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
            encoderMove(2000, 0, 0.5);
            pause(500);
            encoderMove(1500, 1, 0);
        } else if (daWay == 2) {
            telemetry.addLine("DaWay " + daWay);
            telemetry.update();
            encoderMove(1500, 0.8, 0);
        } else if(daWay == 3){
            telemetry.addLine("DaWay " + daWay);
            telemetry.update();
            encoderMove(2000, 0, -0.5);
            pause(500);
            encoderMove(1500, 1, 0);
        } else{ telemetry.addLine("UH OH BOI: " + path);
            telemetry.update(); }
        //clawOpen();
        telemetry.addLine("DOOOOOOOONNNEEEE " + path);
        telemetry.update();
    }
}