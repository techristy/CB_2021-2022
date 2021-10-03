///////////////////// for testing purpose
package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

//import static org.firstinspires.ftc.teamcode.ConceptTensorFlowObjectDetectionWebcam.LABEL_FIRST_ELEMENT;
//import static org.firstinspires.ftc.teamcode.ConceptTensorFlowObjectDetectionWebcam.LABELS;
//import static org.firstinspires.ftc.teamcode.ConceptTensorFlowObjectDetectionWebcam.TFOD_MODEL_ASSET;

/*
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
*/
//import static org.firstinspires.ftc.teamcode.WebcamTest.VUFORIA_KEY;

@Autonomous(name="BlueCarousel", group="Pushbot")

public class BlueCarousel extends LinearOpMode {

    private static int valQUAD = -1;
    private static int valSingle = -1;
    private static int valZero = -1;

    private static float offsetX = .75f / 8f;//changing this moves the three rects and the three circles left or right, range : (-2, 2) not inclusive
    private static float offsetY = 1.5f / 8f;//changing this moves the three rects and circles up or down, range: (-4, 4) not inclusive

    private static float[] midPos = {4f / 8f + offsetX, 4f / 8f + offsetY};//0 = col, 1 = row
    private static float[] leftPos = {2f / 8f + offsetX, 4f / 8f + offsetY};
    private static float[] rightPos = {6f / 8f + offsetX, 4f / 8f + offsetY};


    //DRIVE, IMU, AND ACCEL CONSTANTS

    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .50, correction, rotation;
    PIDController pidRotate, pidDrive;
    HardwareMecanum robot = new HardwareMecanum();
    private ElapsedTime runtime = new ElapsedTime();
    static final double COUNTS_PER_MOTOR_REV = 2150;    //ANDYMARK Motor Encoder ticks
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.77;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    static double DRIVE_SPEED = 0.9;
    static final double TURN_SPEED = 0.575;
    LogisticFunctions function;


    double servoStartingPosition = 0.5;

    double distanceBetweenBlocks = 9.5;


    //boolean center;

    boolean left;

    boolean right;


    boolean targetVisible;

    double blockPosition;

    boolean values[] = new boolean[3];

    String positionArray[] = null;


    //for 90 degrees
    //bigger turn = 26.5
    //smaller turn = 21.6
    //private VuforiaLocalizer vuforia;
    //private TFObjectDetector tfod;
    //String VUFORIA_KEY =
          //  "  ARGKFNf/////AAABmeWmTIKr70aQrGH7lC6M8xBPdcMfnaNjD/dopWNwdsWuQbrZLFQZZBr/eFBlpHuykY0IY4f9Y34OVFaL4NRxmFd4ghxNkwK3Cjl/4Jo6bf/v+ovD7Tqdf8cT0A3McQF2rxOPE8fsmaC2TfCr8nZquqbbaTZT7bxtuvi8skuLfHg0BNRGaKtEYyPaJ+wdvAcJZ8+2rZ6q+77Ooh2teMYGmJRe+KDD8LmIMn5Jh/r/Lbm9WqjmxuSV6NxwAwpqTPydgJAE/19fXRVbC4+vGWAiiAxd/UIrLxDtgwekkiudCLSa1r1Y8XjtaTeUUWYXl7+iAxkAOX3ZYa84fFrPGnFvYdhjnIuRGo4AgL6dvb/pQEaK ";

    //private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
x         */
       // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

       // parameters.vuforiaLicenseKey = VUFORIA_KEY;
       // parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        //vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.





    @Override
    public void runOpMode() throws InterruptedException {

        teleUpdate("status", "Starting runOpMode");
        robot.init(hardwareMap);
        // robot.changeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.FoundationMoverLeft.setPosition(0.3);    //Pull Position 0.75
        //robot.FoundationMoverRight.setPosition(0.88);
        function = new LogisticFunctions(0.6);
        teleUpdate("status", "Starting runOpMode");
        robot.init(hardwareMap);
        robot.FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //robot.FoundationMoverLeft.setPosition(0.3);    //Pull Position 0.75
        //robot.FoundationMoverRight.setPosition(0.88);
        //robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        //robot.webcamServo.setPosition(0.17);
        teleUpdate("WWWWWWWWWWWWWWWWWWWWWWWWWWWWW", "");
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        teleUpdate("EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEee", "");
        imu.initialize(parameters);
        teleUpdate("QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQqqq", "");
        pidRotate = new PIDController(.003, .00003, 0);
        teleUpdate("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA", "");
        pidDrive = new PIDController(.05, 0, 0);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }
        telemetry.addData("Mode", "waiting for start");
        //telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
//        VuforiaLocalizer.Parameters vparameters = new VuforiaLocalizer.Parameters();
//        String VUFORIA_KEY =
//                "  ARGKFNf/////AAABmeWmTIKr70aQrGH7lC6M8xBPdcMfnaNjD/dopWNwdsWuQbrZLFQZZBr/eFBlpHuykY0IY4f9Y34OVFaL4NRxmFd4ghxNkwK3Cjl/4Jo6bf/v+ovD7Tqdf8cT0A3McQF2rxOPE8fsmaC2TfCr8nZquqbbaTZT7bxtuvi8skuLfHg0BNRGaKtEYyPaJ+wdvAcJZ8+2rZ6q+77Ooh2teMYGmJRe+KDD8LmIMn5Jh/r/Lbm9WqjmxuSV6NxwAwpqTPydgJAE/19fXRVbC4+vGWAiiAxd/UIrLxDtgwekkiudCLSa1r1Y8XjtaTeUUWYXl7+iAxkAOX3ZYa84fFrPGnFvYdhjnIuRGo4AgL6dvb/pQEaK ";

//        vparameters.vuforiaLicenseKey = VUFORIA_KEY;
//        vparameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");
//        VuforiaLocalizer vuforia = null;
        //Instantiate the Vuforia engine
//        vuforia = ClassFactory.getInstance().createVuforia(vparameters);
//        TFObjectDetector tfod;
//        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
//                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
//        tfodParameters.minResultConfidence = 0.8f;
        //tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
//        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
        //Thread.sleep(1000);
        //initVuforia();
        //initTfod();

        //tfod.activate();
        //tfod.setZoom(3, 1.78);

        while (!opModeIsActive()) {

            }
            teleUpdate("" + valZero + "" + valSingle + "" + valQUAD, "");



        //robot.wobbleServo.setPosition(0);
        waitForStart();
        encoderDrive(30, 0.5, "drive");
    }

    public void semiTurn(String type, double angle) {
        //resetAngle();
        if (type.equals("counterclockwise")) {
            long time = System.currentTimeMillis();
            while (getAngle() <= angle & (System.currentTimeMillis() < (time + 6000))) {
                power = ((.75 * 2 * 0.684 / 5.063) * (-Math.pow((((getAngle()) + 2.9) / 37.4), 2) + 4.5 * ((getAngle() + 2.9) / 37.4)) + 0.3) / 1.5;
                if (Math.abs(angle) > 180) {
                    power = power * 1.25;
                }

                telemetry.addLine("power: " + power);
                telemetry.addLine("angle: " + getAngle());
                telemetry.update();
                robot.FL.setPower(power);
                robot.FR.setPower(-power);
                robot.BR.setPower(-power);
                robot.BL.setPower(power);
            }
            robot.FL.setPower(0);
            robot.FR.setPower(0);
            robot.BR.setPower(0);
            robot.BL.setPower(0);
        }
        if (type.equals("clockwise")) {
            long time = System.currentTimeMillis();
            while (getAngle() >= -angle && (System.currentTimeMillis() < (time + 5000))) {
                power = ((.75 * 2 * 0.684 / 5.063) * (-Math.pow((((-getAngle()) + 2.9) / 37.4), 2) + 4.5 * ((-getAngle() + 2.9) / 37.4)) + 0.159) / 2.5;
                if (Math.abs(angle) > 180) {
                    power = power * 1.25;
                }
                telemetry.addLine("" + power);
                telemetry.addLine("" + getAngle());
                telemetry.update();
                robot.FL.setPower(-power);
                robot.FR.setPower(power);
                robot.BR.setPower(power);
                robot.BL.setPower(-power);
            }
            robot.FL.setPower(0);
            robot.FR.setPower(0);
            robot.BR.setPower(0);
            robot.BL.setPower(0);
        }
        //robot.changeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void blockServoControlLeft(boolean control) {
        if (control) {

            //robot.FoundationMoverLeft.setPosition(0.8);    //Pull Position 0.75
        } else {
            //robot.FoundationMoverLeft.setPosition(0.3);    //Pull Position 0.75
        }

    }

    public void blockServoControlRight(boolean control) {
        if (control) {

            //robot.FoundationMoverRight.setPosition(0.2);    //Pull Position 0.75
        } else {
            //robot.FoundationMoverRight.setPosition(0.88);    //Pull Position 0.75
        }

    }

    public void fullTurn(String type) {
        //resetAngle();
        if (type.equals("counterclockwise")) {
            long time = System.currentTimeMillis();
            while ((getAngle() <= 170 & (System.currentTimeMillis() < (time + 6000))) && opModeIsActive()) {
                power = ((.75 * 2 * 0.684 / 5.063) * (-Math.pow((((getAngle()) + 2.9) / 37.4), 2) + 4.5 * ((getAngle() + 2.9) / 37.4)) + 0.159) / 1.5;
                telemetry.addLine("power: " + power);
                telemetry.addLine("angle: " + getAngle());
                telemetry.update();
                robot.FL.setPower(power);
                robot.FR.setPower(-power);
                robot.BR.setPower(-power);
                robot.BL.setPower(power);
            }
            robot.FL.setPower(0);
            robot.FR.setPower(0);
            robot.BR.setPower(0);
            robot.BL.setPower(0);
        }
        if (type.equals("clockwise")) {
            long time = System.currentTimeMillis();
            while (getAngle() >= -170 && (System.currentTimeMillis() < (time + 5000))) {
                power = ((.75 * 2 * 0.684 / 5.063) * (-Math.pow((((-getAngle()) + 2.9) / 37.4), 2) + 4.5 * ((-getAngle() + 2.9) / 37.4)) + 0.159) / 1.5;
                telemetry.addLine("" + power);
                telemetry.addLine("" + getAngle());
                telemetry.update();
                robot.FL.setPower(-power);
                robot.FR.setPower(power);
                robot.BR.setPower(power);
                robot.BL.setPower(-power);
            }
            robot.FL.setPower(0);
            robot.FR.setPower(0);
            robot.BR.setPower(0);
            robot.BL.setPower(0);
        }
        //robot.changeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void halfTurn(String type) {
        telemetry.addLine("performing half turn " + type);
        //resetAngle();
        if (type.equals("counterclockwise")) {
            long time = System.currentTimeMillis();
            telemetry.addLine("time: " + time);
            while (getAngle() <= 87 & (System.currentTimeMillis() < (time + 5000))) {
                power = ((0.75 /*used to be .75*/ * 0.684 / 5.063) * (-Math.pow((((getAngle()) + 6.5) / 19.5), 2) + 4.5 * ((getAngle() + 6.5) / 19.5)) + 0.159) / 1.5;
                telemetry.addLine("power: " + power);
                telemetry.update();
                telemetry.addLine("angle: " + getAngle());
                telemetry.update();
                robot.FL.setPower(power);
                robot.BR.setPower(-power);
                robot.FR.setPower(-power);
                robot.BL.setPower(power);
            }
            robot.FL.setPower(0);
            robot.BR.setPower(0);
            robot.FR.setPower(0);
            robot.BL.setPower(0);
        }
        if (type.equals("clockwise")) {
            long time = System.currentTimeMillis();
            while (getAngle() >= -87 && (System.currentTimeMillis() < (time + 5000))) {
                power = ((.75 * 0.684 / 5.063) * (-Math.pow((((-getAngle()) + 6.5) / 19.5), 2) + 4.5 * ((-getAngle() + 6.5) / 19.5)) + 0.159) / 1.5;
                teleUpdate("" + power, "");
                robot.FL.setPower(-power);
                robot.BR.setPower(power);
                robot.FR.setPower(power);
                robot.BL.setPower(-power);
            }
            robot.FL.setPower(0);
            robot.BR.setPower(0);
            robot.FR.setPower(0);
            robot.BL.setPower(0);
        }
        //robot.changeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void encoderDrive(double inches, double pow, String driveMode) throws InterruptedException {
        if (driveMode.equals("drive")) {
            //settargetposition is inverse
            //if setpower command for backward is -, then getpowers for both are both positive
            pidDrive.setSetpoint(0);
            pidDrive.setOutputRange(0, power);
            pidDrive.setInputRange(-90, 90);
            pidDrive.enable();
            //resetAngle();
            teleUpdate("" + robot.FL.getCurrentPosition() + "   <>   " + robot.FL.getTargetPosition(), "");
            int startPos1 = robot.FL.getCurrentPosition();
            int startPos2 = robot.BL.getCurrentPosition();
            int startPos3 = robot.FR.getCurrentPosition();
            int startPos4 = robot.BR.getCurrentPosition();
            robot.FL.setTargetPosition((int) (inches * COUNTS_PER_INCH));
            robot.BR.setTargetPosition((int) (inches * COUNTS_PER_INCH));
            robot.FR.setTargetPosition((int) (inches * COUNTS_PER_INCH));
            robot.BL.setTargetPosition((int) (inches * COUNTS_PER_INCH));
            double currentPosInches;
            if (inches <= 0) {
                robot.FL.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
                robot.BR.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
                robot.FR.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
                robot.BL.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
                //robot.changeSpeed(power);
                while (robot.FR.getTargetPosition() > robot.FR.getCurrentPosition() ||
                        robot.FL.getTargetPosition() > robot.FL.getCurrentPosition() ||
                        robot.BR.getTargetPosition() > robot.BR.getCurrentPosition() ||
                        robot.BL.getTargetPosition() > robot.BL.getCurrentPosition()) {
                    correction = pidDrive.performPID(getAngle());
                    currentPosInches = ((robot.FL.getCurrentPosition() - startPos1) / COUNTS_PER_INCH);
                    teleUpdate("CURRENTPOSINCHES: " + currentPosInches + "", "");
                    power = function.getPowerAt(currentPosInches, -inches, pow, "drive");
                    robot.FL.setPower(power + correction);
                    robot.BR.setPower(power - correction);
                    robot.FR.setPower(power - correction);
                    robot.BL.setPower(power + correction);
                    teleUpdate("currentPos: " + robot.FL.getCurrentPosition() + "    power: " + power + "    correction: " + correction, "");
                }
            } else {
                //robot.changeSpeed(-power);
                robot.FL.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
                robot.BR.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
                robot.FR.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
                robot.BL.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
                while (robot.FR.getTargetPosition() < robot.FR.getCurrentPosition() ||
                        robot.FL.getTargetPosition() < robot.FL.getCurrentPosition() ||
                        robot.BR.getTargetPosition() < robot.BR.getCurrentPosition() ||
                        robot.BL.getTargetPosition() < robot.BL.getCurrentPosition()) {
                    correction = pidDrive.performPID(getAngle());
                    currentPosInches = ((robot.FL.getCurrentPosition() - startPos1) / COUNTS_PER_INCH * -1);
                    power = -function.getPowerAt(currentPosInches, inches, pow, "drive");
                    robot.FL.setPower((power + correction));
                    robot.BR.setPower((power - correction));
                    robot.FR.setPower((power - correction));
                    robot.BL.setPower((power + correction));
                    teleUpdate("currentPos: " + robot.FL.getCurrentPosition() + "    power: " + power + "    correction: " + correction, "");
                }
            }
            // robot.changeSpeed(0);
        } else if (driveMode.equals("strafe")) {/////LEFT IS POSITIVE
            pidDrive.setSetpoint(0);
            pidDrive.setOutputRange(0, power);
            pidDrive.setInputRange(-90, 90);
            pidDrive.enable();
            //resetAngle();

            int startPos1 = robot.FL.getCurrentPosition();
            int startPos2 = robot.BL.getCurrentPosition();
            int startPos3 = robot.FR.getCurrentPosition();
            int startPos4 = robot.BR.getCurrentPosition();
            robot.FL.setTargetPosition((int) (inches * COUNTS_PER_INCH));
            robot.BR.setTargetPosition((int) (inches * COUNTS_PER_INCH));
            robot.FR.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
            robot.BL.setTargetPosition((int) (-inches * COUNTS_PER_INCH));
//            telemetry.addLine(robot.frontLeft.getTargetPosition()+" <- TARGET");
//            telemetry.addLine(robot.frontLeft.getCurrentPosition()+" <- Current");
//            telemetry.update();
//            Thread.sleep(2000);

            double currentPosInches;
            //power = 0.9;
            //robot.changeSpeed(power);
            if (inches > 0) {
                while (robot.FR.getTargetPosition() < robot.FR.getCurrentPosition() ||
                        robot.FL.getTargetPosition() > robot.FL.getCurrentPosition() ||
                        robot.BR.getTargetPosition() > robot.BR.getCurrentPosition() ||
                        robot.BL.getTargetPosition() < robot.BL.getCurrentPosition()) {
                    correction = pidDrive.performPID(getAngle());
                    currentPosInches = ((robot.FL.getCurrentPosition() - startPos1) / COUNTS_PER_INCH);
                    power = function.getPowerAt(currentPosInches, inches, pow, "strafe") * 1.1;
                    robot.FL.setPower((power + correction));
                    robot.BR.setPower((power - correction));
                    robot.FR.setPower(-(power + correction));
                    robot.BL.setPower(-(power - correction));             //STRAFE
                    teleUpdate("power: " + power, "");
                }
            } else {
                while (robot.FR.getTargetPosition() > robot.FR.getCurrentPosition() ||
                        robot.FL.getTargetPosition() < robot.FL.getCurrentPosition() ||
                        robot.BR.getTargetPosition() < robot.BR.getCurrentPosition() ||
                        robot.BL.getTargetPosition() > robot.BL.getCurrentPosition()) {
                    correction = pidDrive.performPID(getAngle());
                    currentPosInches = ((robot.FL.getCurrentPosition() - startPos1) / COUNTS_PER_INCH * -1);
                    power = -function.getPowerAt(currentPosInches, -inches, pow, "strafe") * 1.1;
                    robot.FL.setPower((power + correction));
                    robot.BR.setPower((power - correction));
                    robot.FR.setPower(-(power + correction));
                    robot.BL.setPower(-(power - correction));             //STRAFE
                }
            }
            //robot.changeSpeed(0);
        }
        //robot.changeMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.changeMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Thread.sleep(100);
    }


    public void teleUpdate(String label, String description) {
        if (robot != null && robot.FL != null && robot.FR != null) {

        }
        telemetry.addLine().addData(label + ": ", description);
        telemetry.update();
    }

    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        return deltaAngle;
    }

}
