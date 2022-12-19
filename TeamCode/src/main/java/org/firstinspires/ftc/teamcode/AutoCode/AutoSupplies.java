package org.firstinspires.ftc.teamcode.AutoCode;

//imports

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;


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


public abstract class AutoSupplies extends LinearOpMode{

    //             < < < [ VARIABLES BEGIN HERE ] > > >             \\
    // Motors, servos, etc.
    protected DcMotor front_left = null;
    protected DcMotor front_right = null;
    protected DcMotor back_left = null;
    protected DcMotor back_right = null;

    protected DcMotor left_Arm_Motor = null;
    protected DcMotor right_Arm_Motor = null;
    protected DcMotor pivot_Arm_Motor = null;

    protected BNO055IMU imu;

    protected ElapsedTime runtime = new ElapsedTime();

    //  Protected variables
    protected double globalAngle;
    protected double globalPitch;
    protected Orientation lastAngles = new Orientation();
    protected Orientation lastPitches = new Orientation();


    /*
    protected Rev2mDistanceSensor distanceFwdLeft = null;
    protected Rev2mDistanceSensor distanceFwdRight = null;
    protected Rev2mDistanceSensor distanceBackLeft = null;
    protected Rev2mDistanceSensor distanceBackRight = null;
    protected Rev2mDistanceSensor distanceLeftTop = null;
    protected Rev2mDistanceSensor distanceLeftBottom = null;
*/

    //Encoder Values
    //Neverest 40 motor spec: quadrature encoder, 7 pulses per revolution, count = 7 * 40
    private static final double COUNTS_PER_MOTOR_REV = 420; // Neverest 40 motor encoder - orginal val = 280
    private static final double DRIVE_GEAR_REDUCTION = 1; // This is < 1 if geared up
    private static final double COUNTS_PER_DEGREE1 = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / 360;
    //---callable methods---\\

    /*public void setup()
    {
        resetArmEncoders();
    }*/

    private static final double A_Left = 1;
    private static final double A_Top = 1; //Fill in with legit values
    private static final double A_Right = 320;
    private static final double A_Bottom = 480;

    private static final double B_Left = 321;
    private static final double B_Top = 1; //Fill in with legit values
    private static final double B_Right = 640;
    private static final double B_Bottom = 480;

    private static final double C_Left = 0;
    private static final double C_Top = 0; //Fill in with legit values
    private static final double C_Right = 0;
    private static final double C_Bottom = 0;


    private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";

    private static final String[] LABELS = {
            "1 Bolt",
            "2 Bulb",
            "3 Panel"
    };

    private static final String VUFORIA_KEY =
            "AcuKVGn/////AAABmYltM2e7pk7PgntBn1QBuWhrTu52XxsTTE0NUewmsJRB/KPGYohT+y+YHK8/LEcyIki6iW/Msvl8c4kwg5XFMGWY19pEmKxhqgtXd58d5Kyu+UC8NFYlEf52vluc0yibUUmianPhos+tutELIA2PqQqSgM3WTXxH+fwUdPIvAJuTHZqxu9t9cK3qKZeLzsgDSOXeKEjOcmkJQsFsneppUGPTrehHCDLRYyOAwDgHsr2X65ZhZ4cOSYoQR0WDo2ogRvFxuWDPinD9Gj8jq4AxNDrAr3kLkZwbtSc6IfD0dAysXtIoK41JgDZNK2BG93C9tDoNmNTUJNFv3pn6+cywIch5e1ylQ0ZBfZ47ZMCTDJA+";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    // //             < < < [ FUNCTIONS BEGIN HERE ] > > >             \\ \\

    //move
    public void move(long millis, double x, double y)
    {
        double fwdBackPower = y;
        double strafePower = x;
        double leftFrontPower = fwdBackPower + strafePower;
        double rightFrontPower = fwdBackPower - strafePower;
        double leftBackPower = fwdBackPower - strafePower;
        double rightBackPower = fwdBackPower + strafePower;
        double maxPower;
        double max = 1.0;

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
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() <= millis) {
            front_left.setPower(leftFrontPower*max);
            front_right.setPower(rightFrontPower*max);
            back_left.setPower(leftBackPower*max);
            back_right.setPower(rightBackPower*max);
        }
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
    }
    public void encoderMove(double degrees, double x, double y){
        resetDriveEncoders();
        double counts = degrees * COUNTS_PER_DEGREE1;
        double fwdBackPower = y;
        double strafePower = x;
        double leftFrontPower = fwdBackPower + strafePower;
        double rightFrontPower = fwdBackPower - strafePower;
        double leftBackPower = fwdBackPower - strafePower;
        double rightBackPower = fwdBackPower + strafePower;
        double maxPower;
        double max = 1.0;
        double posPower = 0.2;
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
        double averageEnc = (Math.abs(front_left.getCurrentPosition())
                + Math.abs(front_right.getCurrentPosition())
                + Math.abs(back_left.getCurrentPosition())
                + Math.abs(back_right.getCurrentPosition()))/4.0;
        while (opModeIsActive() && averageEnc <= counts){
            averageEnc = (Math.abs(front_left.getCurrentPosition())
                    + Math.abs(front_right.getCurrentPosition())
                    + Math.abs(back_left.getCurrentPosition())
                    + Math.abs(back_right.getCurrentPosition()))/4.0;
            if(posPower < 1 && averageEnc/counts < .6){
                posPower *= 1.1;
            }
            else if(posPower >= 1 && averageEnc/counts <.6){
                posPower = 1;
            }
            else if(averageEnc/counts >= .6 && posPower >= .25){
                posPower *= .99;
            }
            else{
                posPower = .25;
            }
            front_left.setPower(leftFrontPower*max*posPower);
            back_left.setPower(leftBackPower*max*posPower);
            front_right.setPower(rightFrontPower*max*posPower);
            back_right.setPower(rightBackPower*max*posPower);
        }
        front_left.setPower(0);
        back_left.setPower(0);
        front_right.setPower(0);
        back_right.setPower(0);
    }
    public void resetDriveEncoders(){
        front_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void setPower(double x, double y)
    {
        double fwdBackPower = y;
        double strafePower = x;
        double leftFrontPower = fwdBackPower + strafePower;
        double rightFrontPower = fwdBackPower - strafePower;
        double leftBackPower = fwdBackPower - strafePower;
        double rightBackPower = fwdBackPower + strafePower;

        front_left.setPower(leftFrontPower);
        front_right.setPower(rightFrontPower);
        back_left.setPower(leftBackPower);
        back_right.setPower(rightBackPower);
    }
    public void turn(int degrees, double power){
        int left = 1;
        int right = 1;
        resetAngle();
        telemetry.addData("Angle",getAngle());
        telemetry.update();
        if(degrees >= 0){
            right *= -1;
        }
        else if(degrees < 0){
            left *= -1;
        }

        back_left.setPower(power * left);
        front_left.setPower(power * left);
        back_right.setPower(power * right);
        front_right.setPower(power* right);

        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() >= degrees) {telemetry.addData("Angle2",getAngle());telemetry.update();}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {telemetry.addData("Angle2",getAngle());telemetry.update();}

        // turn the motors off.
        back_left.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        front_right.setPower(0);
    }
    //Using the gyroscope, when a degree is passed both left and right motors move accordingly in
    //order to turn the robot to the right or left until the bearing is equal to or greater than the
    //specified degree. Power can also be specified.
    //used commonly in pairs(one fast for speed and one slow for accuracy) to improve movement time.
    public void turnToS(int degrees, double power, int loopnum){
        int left = 1;
        int right = 1;
        double distance = getAngle() - degrees;
        double startAngle = getAngle();
        telemetry.addData("Angle3",getAngle());
        telemetry.update();
        if(getAngle() <= degrees){
            left *= -1;
        }
        else if(getAngle() > degrees){
            right *= -1;
        }

        back_left.setPower(power * left);
        front_left.setPower(power * left);
        back_right.setPower(power * right);
        front_right.setPower(power* right);

        if (getAngle() > degrees)
        {
            // On left turn we have to get off zero first.
            while (opModeIsActive() && getAngle() >= degrees) {
                //telemetry.addData("Angle4",getAngle());
                //telemetry.update();
                if((startAngle + ((distance/4)*3)) > getAngle()){
                    left *= 1.05;
                    right *= 1.05;
                }
                else{
                    if(left > 1 || left < -1 || right > 1 || right < -1){
                        left*=0.95;
                        right*=0.95;
                    }
                }
            }
        }
        else {    // right turn.
            while (opModeIsActive() && getAngle() <= degrees) {
                //telemetry.addData("Angle4", getAngle());
                //telemetry.update();
                if((startAngle + ((distance/4)*3)) > getAngle()){
                    left *= 1.05;
                    right *= 1.05;
                }
                else{
                    if(left > 1 || left < -1 || right > 1 || right < -1){
                        left*=0.95;
                        right*=0.95;
                    }
                }
            }
        }
        // turn the motors off.
        back_left.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        front_right.setPower(0);
        if(--loopnum > 0){
            turnToS(degrees, power/2, loopnum);
        }
    }
    //Using the gyroscope, when a degree is passed both left and right motors move accordingly in
    //order to turn the robot to the right or left until the bearing is equal to or greater than the
    //specified degree. Power can also be specified.
    //used commonly in pairs(one fast for speed and one slow for accuracy) to improve movement time.
    public void turnTo(int degrees, double power){
        int left = 1;
        int right = 1;
        telemetry.addData("Angle3",getAngle());
        telemetry.update();
        if(getAngle() >= degrees){
            left *= -1;
        }
        else if(getAngle() < degrees){
            right *= -1;
        }

        back_left.setPower(power * left);
        front_left.setPower(power * left);
        back_right.setPower(power * right);
        front_right.setPower(power* right);

        if (getAngle() > degrees)
        {
            // On left turn we have to get off zero first.
            while (opModeIsActive() && getAngle() >= degrees) {
                //telemetry.addData("Angle4",getAngle());
                //telemetry.update();
            }
        }
        else {    // right turn.
            while (opModeIsActive() && getAngle() <= degrees) {
                //telemetry.addData("Angle4", getAngle());
                //telemetry.update();
            }
        }
        // turn the motors off.
        back_left.setPower(0);
        front_left.setPower(0);
        back_right.setPower(0);
        front_right.setPower(0);
    }
    public void alignDistanceFwd(){

    }
    //  Pause for the specified amount of time (time: mili secs)
    public void pause(long millis){
        runtime.reset();
        while (opModeIsActive() && runtime.milliseconds() <= millis){

        }
    }
    //Resets gyro sensor bearing value to 0
    //commonly used to calibrate before a match as well
    public void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
    //Resets gyro sensor pitch value to 0
    //commonly used to calibrate before a match as well
    public void resetPitch()
    {
        lastPitches = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalPitch = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */


    //uses the imu to find the current angle
    public double getAngle()
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
    //uses the imu to get the current pitch of the robot
    public double getPitch() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.secondAngle - lastPitches.secondAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalPitch += deltaAngle;

        lastPitches = angles;

        return globalPitch;
    }
    /*
    public double getDistanceFwdLeft(){
        return distanceFwdLeft.getDistance(DistanceUnit.MM);
    }
    public double getDistanceFwdRight(){
        return distanceFwdRight.getDistance(DistanceUnit.MM);
    }
    public double getDistanceBackLeft(){
        return distanceBackRight.getDistance(DistanceUnit.MM);
    }
    public double getDistanceBackRight(){
        return distanceBackRight.getDistance(DistanceUnit.MM);
    }
    public double getDistanceLeftTop(){ return distanceLeftTop.getDistance(DistanceUnit.MM); }
    public double getDistanceLeftBottom(){ return distanceLeftBottom.getDistance(DistanceUnit.MM); }
    public void moveUsingLeftDistance(double distance, double power){//- means strafe left and + means strafe right
        while (opModeIsActive()) {
            double dist = distanceLeftTop.getDistance(DistanceUnit.MM);
            double dist2 = distanceLeftBottom.getDistance(DistanceUnit.MM);
            //time out check
            if(dist == 65535 || distanceLeftTop.didTimeoutOccur()) {
                dist = 65535;
            }
            if(dist2 == 65535 || distanceLeftBottom.didTimeoutOccur()){
                dist2 = 65535;
            }
            if(dist >= 8000 || dist2 >= 8000){
                dist = 8190;
                dist2 = 8190;
            }
            //telemetry
            telemetry.addData("distance val 1", dist);
            telemetry.addData("distance val 2", dist2);
            //power set or loop break if distance traveled is met
            if(power <= 0) {
                if (dist != 65535 && dist2 != 65535 && (dist + dist2) / 2 > distance) {
                    setPower(power, 0);
                } else if (dist != 65535 && dist2 == 65535 && dist > distance) {
                    setPower(power, 0);
                } else if (dist == 65535 && dist2 != 65535 && dist2 > distance) {
                    setPower(power, 0);
                } else {
                    break;
                }
            }
            else {
                if (dist != 65535 && dist2 != 65535 && (dist + dist2) / 2 < distance) {
                    setPower(power, 0);
                } else if (dist != 65535 && dist2 == 65535 && dist < distance) {
                    setPower(power, 0);
                } else if (dist == 65535 && dist2 != 65535 && dist2 < distance) {
                    setPower(power, 0);
                } else {
                    break;
                }
            }
            telemetry.update();
        }
    }
    public void moveUsingFwdDistance(double distance, double power){//- means strafe back and + means strafe fwd
        while (opModeIsActive()) {
            double dist = distanceFwdLeft.getDistance(DistanceUnit.MM);
            double dist2 = distanceFwdRight.getDistance(DistanceUnit.MM);
            //time out check
            if(dist == 65535 || distanceFwdLeft.didTimeoutOccur()) {
                dist = 65535;
            }
            if(dist2 == 65535 || distanceFwdRight.didTimeoutOccur()){
                dist2 = 65535;
            }
            if(dist >= 8190 || dist2 >= 8190){
                dist = 8190;
                dist2 = 8190;
            }
            //telemetry
            telemetry.addData("distance val 1", dist);
            telemetry.addData("distance val 2", dist2);
            //power set or loop break if distance traveled is met
            if(power <= 0) {
                if (dist != 65535 && dist2 != 65535 && (dist + dist2) / 2 < distance) {
                    setPower(0, power);
                } else if (dist != 65535 && dist2 == 65535 && dist < distance) {
                    setPower(0, power);
                } else if (dist == 65535 && dist2 != 65535 && dist2 < distance) {
                    setPower(0, power);
                } else {
                    break;
                }
            }
            else {
                if (dist != 65535 && dist2 != 65535 && (dist + dist2) / 2 > distance) {
                    setPower(0, power);
                } else if (dist != 65535 && dist2 == 65535 && dist > distance) {
                    setPower(0, power);
                } else if (dist == 65535 && dist2 != 65535 && dist2 > distance) {
                    setPower(0, power);
                } else {
                    break;
                }
            }
            telemetry.update();
        }
    }
*/
    public void initForAutonomous()
    {
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        BNO055IMU.Parameters gyroParameters = new BNO055IMU.Parameters();

        gyroParameters.mode                = BNO055IMU.SensorMode.IMU;
        gyroParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        gyroParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyroParameters.loggingEnabled      = false;

// Connect Motors to Phone \\
        //initialize hardware
        //main motors
        front_right = hardwareMap.get(DcMotor.class, "front_right");
        front_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_left = hardwareMap.get(DcMotor.class, "back_left");
        back_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        front_left = hardwareMap.get(DcMotor.class, "front_left");
        front_left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        back_right = hardwareMap.get(DcMotor.class, "back_right");
        back_right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        back_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        back_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        front_left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//sensors
        /*
        distanceFwdLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distanceFwdLeft");
        distanceFwdRight = hardwareMap.get(Rev2mDistanceSensor.class, "distanceFwdRight");
        distanceBackLeft = hardwareMap.get(Rev2mDistanceSensor.class, "distanceBackLeft");
        distanceBackRight = hardwareMap.get(Rev2mDistanceSensor.class, "distanceBackRight");
        distanceLeftTop = hardwareMap.get(Rev2mDistanceSensor.class, "distanceLeftTop");
        distanceLeftBottom = hardwareMap.get(Rev2mDistanceSensor.class, "distanceLeftBottom");
         */




// Set the direction for each of the motors \\
        front_left.setDirection(DcMotor.Direction.REVERSE);
        back_left.setDirection(DcMotor.Direction.REVERSE);




        //resetArmEncoders();

        //initializes imu and calibrates it. Prepares lift motor to land using the encoder
        // Lights turn green when it is calibrated
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(gyroParameters);

        telemetry.clear();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void test()
    {
        //telemetry.update();
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        telemetry.addData("InitVIEW here", "AHHHHHHH");
        telemetry.update();
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        telemetry.addData("InitVIEW here", "AHHHHHH init-ed");
        telemetry.update();
        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
        telemetry.addData("InitVIEW here", "AH Config-ed");
        telemetry.update();
        telemetry.addData("Param test here", parameters);
        telemetry.update();
        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        telemetry.addData("InitVIEW here", "AH class time");
        telemetry.update();
        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }


    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }

    public void initVision(){
        telemetry.addData("InitVision here", "HUUZZZUH");
        telemetry.update();
        initVuforia();
        initTfod();
        telemetry.addData("InitVision here", "HUUZZZUH part 2");
        telemetry.update();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
        }
    }

    public Recognition getConeType(){
        if (tfod != null) {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                for (Recognition recognition : updatedRecognitions) {
                    int i = 0;
                    if(recognition.getLabel() == LABELS[i] && i < 3) {
                        return recognition;
                    }
                    i++;
                }
                //telemetry.update();
            }
        }
        return null;
    }

    public int getZone(Recognition cone){
        if(cone == null){
            return 12345;
        }
        if(cone.getLabel().equals("1 Bolt")){
            return 1;
        }
        else if(cone.getLabel().equals("2 Bulb")){
            return 2;
        }
        else if(cone.getLabel().equals("3 Panel")){
            return 3;
        } else{ return -1; }
    }

    // //             < < < [ DIST SENSOR CODE BEGIN HERE ] > > >             \\ \\

    /*
    public double getDistanceFwdLeft(){
        return distanceFwdLeft.getDistance(DistanceUnit.MM);
    }
    public double getDistanceFwdRight(){
        return distanceFwdRight.getDistance(DistanceUnit.MM);
    }
    public double getDistanceBackLeft(){
        return distanceBackRight.getDistance(DistanceUnit.MM);
    }
    public double getDistanceBackRight(){
        return distanceBackRight.getDistance(DistanceUnit.MM);
    }
    public double getDistanceLeftTop(){ return distanceLeftTop.getDistance(DistanceUnit.MM); }
    public double getDistanceLeftBottom(){ return distanceLeftBottom.getDistance(DistanceUnit.MM); }
    public void moveUsingLeftDistance(double distance, double power){//- means strafe left and + means strafe right
        while (opModeIsActive()) {
            double dist = distanceLeftTop.getDistance(DistanceUnit.MM);
            double dist2 = distanceLeftBottom.getDistance(DistanceUnit.MM);
            //time out check
            if(dist == 65535 || distanceLeftTop.didTimeoutOccur()) {
                dist = 65535;
            }
            if(dist2 == 65535 || distanceLeftBottom.didTimeoutOccur()){
                dist2 = 65535;
            }
            if(dist >= 8000 || dist2 >= 8000){
                dist = 8190;
                dist2 = 8190;
            }
            //telemetry
            telemetry.addData("distance val 1", dist);
            telemetry.addData("distance val 2", dist2);
            //power set or loop break if distance traveled is met
            if(power <= 0) {
                if (dist != 65535 && dist2 != 65535 && (dist + dist2) / 2 > distance) {
                    setPower(power, 0);
                } else if (dist != 65535 && dist2 == 65535 && dist > distance) {
                    setPower(power, 0);
                } else if (dist == 65535 && dist2 != 65535 && dist2 > distance) {
                    setPower(power, 0);
                } else {
                    break;
                }
            }
            else {
                if (dist != 65535 && dist2 != 65535 && (dist + dist2) / 2 < distance) {
                    setPower(power, 0);
                } else if (dist != 65535 && dist2 == 65535 && dist < distance) {
                    setPower(power, 0);
                } else if (dist == 65535 && dist2 != 65535 && dist2 < distance) {
                    setPower(power, 0);
                } else {
                    break;
                }
            }
            telemetry.update();
        }
    }
    public void moveUsingFwdDistance(double distance, double power){//- means strafe back and + means strafe fwd
        while (opModeIsActive()) {
            double dist = distanceFwdLeft.getDistance(DistanceUnit.MM);
            double dist2 = distanceFwdRight.getDistance(DistanceUnit.MM);
            //time out check
            if(dist == 65535 || distanceFwdLeft.didTimeoutOccur()) {
                dist = 65535;
            }
            if(dist2 == 65535 || distanceFwdRight.didTimeoutOccur()){
                dist2 = 65535;
            }
            if(dist >= 8190 || dist2 >= 8190){
                dist = 8190;
                dist2 = 8190;
            }
            //telemetry
            telemetry.addData("distance val 1", dist);
            telemetry.addData("distance val 2", dist2);
            //power set or loop break if distance traveled is met
            if(power <= 0) {
                if (dist != 65535 && dist2 != 65535 && (dist + dist2) / 2 < distance) {
                    setPower(0, power);
                } else if (dist != 65535 && dist2 == 65535 && dist < distance) {
                    setPower(0, power);
                } else if (dist == 65535 && dist2 != 65535 && dist2 < distance) {
                    setPower(0, power);
                } else {
                    break;
                }
            }
            else {
                if (dist != 65535 && dist2 != 65535 && (dist + dist2) / 2 > distance) {
                    setPower(0, power);
                } else if (dist != 65535 && dist2 == 65535 && dist > distance) {
                    setPower(0, power);
                } else if (dist == 65535 && dist2 != 65535 && dist2 > distance) {
                    setPower(0, power);
                } else {
                    break;
                }
            }
            telemetry.update();
        }
    }
*/
}