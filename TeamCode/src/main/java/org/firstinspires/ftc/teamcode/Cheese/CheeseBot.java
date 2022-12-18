package org.firstinspires.ftc.teamcode.Cheese;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Cheesy Boi", group="Cheese")
//@Disabled
public class CheeseBot extends LinearOpMode {
    //All hardware
    protected DcMotor front_left = null;
    protected DcMotor front_right = null;
    protected DcMotor back_left = null;
    protected DcMotor back_right = null;
    //protected Rev2mDistanceSensor distanceLeft = null;
    //protected Rev2mDistanceSensor distanceRight = null;
    //protected ColorSensor colorLeft = null;
    //protected Servo servo = null;

    //Variables
    private double max = 1.0;
    double maxPower;
    double powerLim = 1;
    double moveDir = -1;
    double distance1 = 0;
    double distance2 = 0;
    double color1 = 0;
    double blue = 0;
    double green = 0;
    double red = 0;
    double servoPos = .5;

    @Override
    public void runOpMode() {
        //Prepares all the hardware
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);
        //distanceLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distanceLeft");
        //distanceRight = hardwareMap.get(Rev2mDistanceSensor.class, "distanceRight");
        //colorLeft = hardwareMap.get(ColorSensor.class, "colorLeft");
        //servo = hardwareMap.get(Servo.class, "servo");
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //variables
        boolean changed3 = false;
        boolean changed4 = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //collects input from the joysticks
            double fwdBackPower = this.gamepad1.left_stick_y * moveDir;
            double strafePower = this.gamepad1.left_stick_x * moveDir;
            double turnPower = -this.gamepad1.right_stick_x;

            //does math to figure the power that should be applied to every motor
            double leftFrontPower = (fwdBackPower - turnPower - strafePower)*powerLim;
            double rightFrontPower = (fwdBackPower + turnPower + strafePower)*powerLim;
            double leftBackPower = (fwdBackPower - turnPower + strafePower)*powerLim;
            double rightBackPower = (fwdBackPower + turnPower - strafePower)*powerLim;


            maxPower = Math.abs(leftFrontPower);
            if (Math.abs(rightFrontPower) > maxPower) {
                maxPower = Math.abs(rightFrontPower);
            }
            if (Math.abs(leftBackPower) > maxPower) {
                maxPower = Math.abs(leftBackPower);
            }
            if (Math.abs(rightBackPower) > maxPower) {
                maxPower = Math.abs(rightBackPower);
            }
            if (maxPower > 1) {
                leftFrontPower = leftFrontPower / maxPower;
                rightFrontPower = rightFrontPower / maxPower;
                leftBackPower = leftBackPower / maxPower;
                rightBackPower = rightBackPower / maxPower;
            }
            //sets the power of the motors
            front_left.setPower(leftFrontPower*max);
            front_right.setPower(rightFrontPower*max);
            back_left.setPower(leftBackPower*max);
            back_right.setPower(rightBackPower*max);


            //toggle for speed and direction of the bot for easier control
            if(gamepad1.b && !changed3) {//speed limiter toggle
                if(powerLim == .5){
                    powerLim = 1;
                }
                else{
                    powerLim = .5;
                }
                changed3 = true;
            } else if(!gamepad1.b){changed3 = false;}

            if(gamepad1.a && !changed4) {//direction change toggle
                if(moveDir == 1){
                    moveDir = -1;
                }
                else{
                    moveDir = 1;
                }
                changed4 = true;
            } else if(!gamepad1.a){changed4 = false;}

            if(gamepad1.dpad_up){//servo controls
                servoPos += .01;
                if(servoPos < 0){
                    servoPos = 0;
                }
                else if(servoPos > 1){
                    servoPos = 1;
                }
            }
            else if(gamepad1.dpad_down){
                servoPos -= .01;
                if(servoPos < 0){
                    servoPos = 0;
                }
                else if(servoPos > 1){
                    servoPos = 1;
                }
            }
           // servo.setPosition(servoPos);
            //collects data from sensors
            //distance1 = distanceLeft.getDistance(DistanceUnit.MM);
            //distance2 = distanceRight.getDistance(DistanceUnit.MM);
            //color1 = colorLeft.alpha();
            //red = colorLeft.red();
            //green = colorLeft.green();
            //blue = colorLeft.blue();

            //all telemetry
            telemetry.addData("Wheel Position", front_left.getCurrentPosition()); //to be used when the encoders are ready
            telemetry.addData("Max Speed",powerLim);
            telemetry.addData("Direction",moveDir);
            telemetry.addData("Distance1 in mm", distance1);
            telemetry.addData("Distance2 in mm", distance2);
            telemetry.addData("Alpha", color1);
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Servo Position", servoPos);
            telemetry.update();
        }
    }
}
