package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.State;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

@Autonomous(name="HighGoalAuto", group="Linear Opmode")

public class HighGoalAuto extends LinearOpMode {
    public State state;
    private ElapsedTime runtime = new ElapsedTime();

    //Drivetrain
    private DcMotor driveFrontRight;
    private DcMotor driveFrontLeft;
    private DcMotor driveBackRight;
    private DcMotor driveBackLeft;

    //shooter
    private DcMotor shooterWheel;
    private Servo shooterFlicker;
    boolean changeFlicker = true;
    private float flickerPos = 0;

    //indexing
    private DcMotor index;
    private Servo indexRight;
    private Servo indexLeft;

    //intake motor
    private DcMotor intake;

    //wobble goal
    private DcMotor wobbleLead;
    private Servo wobblePivotTop;
    private Servo wobblePivotBottom;
    private Servo wobbleThirdPivot;
    private Servo wobbleClaw;
    private double clawPos = 1;

    private double drive, strafe = 0;

    //ring detection
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private int ringHeight;

    private static final String VUFORIA_KEY =
            "AfDxTOz/////AAABmZP0ZciU3EdTii04SAkq0dI8nEBh4mM/bXMf3H6bRJJbH/XCSdLIe5SDSavwPb0wJvUdnsmXcal43ZW2YJRG6j65bfewYJPCb+jGn7IW7kd5rKWs11G7CtFSMGEOhA5NU8gi39eHW0pmXC8NEXBn3CmK67TIENGm/YBN6f+xmkmDvBQjaJc2hJ93HPvhAnIiAbJT9/fWijwg9IovTok/xAcAcuIKz3XK/lnJXu6XdJ1MyRtoXO7yf1W4ReDHngWCtKI9B7bAnD6zPNhZoVLVzl34E8XKed/dGShIoCmIUTe0HoUniP0ye3AnwhFgxLhgPcysF8uVqKN0VKBpDH1zU7J7keZdjWHM6jvn29oLMK7W";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction;

    private enum State {
        TOSCAN,
        SCAN,
        TOSHOOT,
        SHOOT1,
        SHOOT2,
        SHOOT3,
        DELIVER,
        INTAKE,
        TOSHOOT2,
        SHOOT4,
        SHOOT5,
        SHOOT6,
        POWERSHOTS,
        TOSECONDWOBBLE,
        DELIVER2,
        PARK,
        STOP,
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Drivetrain
        driveFrontRight = hardwareMap.get(DcMotor.class, "driveFrontRight");
        driveFrontLeft = hardwareMap.get(DcMotor.class, "driveFrontLeft");
        driveBackRight = hardwareMap.get(DcMotor.class, "driveBackRight");
        driveBackLeft = hardwareMap.get(DcMotor.class, "driveBackLeft");

        //indexing
        index = hardwareMap.get(DcMotor.class, "index");
        indexRight = hardwareMap.get(Servo.class, "indexRight");
        indexLeft = hardwareMap.get(Servo.class, "indexLeft");
        indexLeft.setPosition(0);
        indexRight.setPosition(1);

        //shooter
        shooterWheel = hardwareMap.get(DcMotor.class, "shooterWheel");
        shooterFlicker = hardwareMap.get(Servo.class, "shooterFlicker");
        shooterFlicker.setPosition(.3);

        //intake
        intake = hardwareMap.get(DcMotor.class, "intake");

        //wobble goal
        wobbleLead = hardwareMap.get(DcMotor.class, "wobbleLead");
        wobblePivotTop = hardwareMap.get(Servo.class, "wobblePivotTop");
        wobblePivotBottom = hardwareMap.get(Servo.class, "wobblePivotBottom");
        wobbleThirdPivot = hardwareMap.get(Servo.class, "wobbleThirdPivot");
        wobbleClaw = hardwareMap.get(Servo.class, "wobbleClaw");
        wobblePivotTop.setPosition(0);
        wobblePivotBottom.setPosition(1);
        wobbleThirdPivot.setPosition(0);
        wobbleClaw.setPosition(clawPos);

        //ring detection
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1.0, 16.0/9.0);
        }

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        //parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU.
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        resetEncoders();
        useEncoders();
        resetAngle();

        if (tfod != null) {
            tfod.activate();
        }

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("pos", driveFrontLeft.getCurrentPosition());
        telemetry.addData("angle:", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        telemetry.update();

        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);
        shooterWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.update();
        waitForStart();
        resetEncoders();
        useEncoders();
        state = State.TOSCAN;
        runtime.reset();
        ElapsedTime autoTime = new ElapsedTime();
        while (opModeIsActive())
        {
            telemetry.addData("time", autoTime.seconds());
            telemetry.update();
            switch(state) {
                case TOSCAN:
                    shooterWheel.setPower(-.43);
                    resetEncoders();
                    useEncoders();
                    encoderForwards(16, .5);
                    setStateRunning(State.SCAN);
                    break;
                case SCAN:
                    shooterWheel.setPower(-.43);
                    if (tfod != null) {
                        while(autoTime.seconds() < 3)
                        {
                            shooterWheel.setPower(-.43);
                            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                            if (updatedRecognitions != null) {
                                telemetry.addData("# Object Detected", updatedRecognitions.size());
                                //int i = 0;
                                for (Recognition r : updatedRecognitions) {
                                    if(r.getLabel().equals("Quad"))
                                    {
                                        ringHeight = 4;
                                    }
                                    else if(r.getLabel().equals("Single"))
                                    {
                                        ringHeight = 1;
                                    }
                                    else
                                    {
                                        ringHeight = 0;
                                    }
                                    telemetry.addData("# of rings", ringHeight);
                                    telemetry.update();
                                }

                            }
                        }
                    }
                    setStateRunning(State.TOSHOOT);
                    break;
                case TOSHOOT:
                    shooterWheel.setPower(-.45);
                    //shooterFlicker.setPosition(.3);
                    resetEncoders();
                    useEncoders();
                    encoderCrab(5, -0.6);
                    resetEncoders();
                    useEncoders();
                    encoderForwards(40, .6);
                    //turnDegrees(1, .4);
                    setStateRunning(State.SHOOT1);
                    break;
                case SHOOT1:
                    shooterWheel.setPower(-.48);
                    flicker();
                    runtime.reset();
                    while (runtime.seconds()<0.8)
                    {
                    }
                    setStateRunning(State.SHOOT2);
                    break;
                case SHOOT2:
                    shooterWheel.setPower(-.41);
                    flicker();
                    runtime.reset();
                    while (runtime.seconds()<0.8)
                    {
                    }
                    setStateRunning(State.SHOOT3);
                    break;
                case SHOOT3:
                    shooterWheel.setPower(-.48);
                    flicker();
                    runtime.reset();
                    while (runtime.seconds()<0.8)
                    {
                    }
                    setStateRunning(State.DELIVER);
                    break;
                case DELIVER:
                    if(ringHeight==0)
                    {
                        resetEncoders();
                        useEncoders();
                        encoderCrab(5, .6);
                        /*encoderForwards(15, .7);
                        turnDegrees(98,.45);
                        resetEncoders();
                        useEncoders();
                        encoderForwards(1, .65);*/
                        clawPos=1;
                        wobbleClaw.setPosition(clawPos);
                        wobblePivotTop.setPosition(1);
                        wobblePivotBottom.setPosition(0);
                        wobbleThirdPivot.setPosition(1);
                        sleep(200);
                        //wobbleClaw.setPosition(.75);
                        resetEncoders();
                        useEncoders();
                        //encoderCrab(15, -0.6);
                        setStateRunning(State.STOP);
                    }
                    else if(ringHeight==1)
                    {
                        //turnDegrees(5,.5);
                        resetEncoders();
                        useEncoders();
                        encoderCrab(13, .6);
                        resetEncoders();
                        useEncoders();
                        encoderForwards(15, .7);
                        clawPos=1;
                        wobbleClaw.setPosition(clawPos);
                        wobblePivotTop.setPosition(1);
                        wobblePivotBottom.setPosition(0);
                        wobbleThirdPivot.setPosition(1);
                        sleep(300);
                        //wobbleClaw.setPosition(.75);
                        setStateRunning(State.STOP);
                        //setStateRunning(State.TOSECONDWOBBLE);
                    }
                    else if(ringHeight==4)
                    {
                        resetEncoders();
                        useEncoders();
                        encoderCrab(20, -.6);
                        resetEncoders();
                        useEncoders();
                        encoderForwards(35, .7);
                        clawPos=1;
                        wobbleClaw.setPosition(clawPos);
                        wobblePivotTop.setPosition(1);
                        wobblePivotBottom.setPosition(0);
                        wobbleThirdPivot.setPosition(1);
                        sleep(500);
                        setStateRunning(State.INTAKE);
                    }
                    break;
                case INTAKE:
                    if(ringHeight==4){
                        clawPos=.75;
                        wobbleClaw.setPosition(clawPos);
                        resetEncoders();
                        useEncoders();
                        encoderCrab(22, .6);
                        //get intake down
                        resetEncoders();
                        useEncoders();
                        encoderBackwards(10, .7);
                        resetEncoders();
                        useEncoders();
                        encoderForwards(7, .7);
                        resetEncoders();
                        useEncoders();
                        //drive to intake
                        intake.setPower(1);
                        index.setPower(-1);
                        encoderBackwards(60, .7);
                        intake.setPower(0);
                        setStateRunning(State.TOSHOOT2);
                    }
                    else{
                        setStateRunning(State.TOSECONDWOBBLE);
                    }
                    break;
                case TOSHOOT2:
                    shooterWheel.setPower(-.47);
                    index.setPower(-1);
                    intake.setPower(-1);
                    shooterWheel.setPower(-.485);
                    //wobbleClaw.setPosition(1);
                    wobblePivotTop.setPosition(.075);
                    wobblePivotBottom.setPosition(.925);
                    wobbleThirdPivot.setPosition(.075);
                    resetEncoders();
                    useEncoders();
                    encoderForwards(30, 0.8);
                    intake.setPower(0);
                    resetEncoders();
                    useEncoders();
                    encoderCrab(11, -0.6);
                    //turnDegrees(-5,0.35);
                    index.setPower(0);
                    setStateRunning(State.SHOOT4);
                    break;
                case SHOOT4:
                    shooterWheel.setPower(-.47);
                    flicker();
                    runtime.reset();
                    while (runtime.seconds()<0.8)
                    {
                    }
                    //setStateRunning(State.PARK);
                    setStateRunning(State.SHOOT5);
                    break;
                case SHOOT5:
                    shooterWheel.setPower(-.41);
                    flicker();
                    runtime.reset();
                    while (runtime.seconds()<0.8)
                    {
                    }
                    setStateRunning(State.SHOOT6);
                    break;
                case SHOOT6:
                    shooterWheel.setPower(-.48);
                    flicker();
                    runtime.reset();
                    while (runtime.seconds()<0.8)
                    {
                    }
                    setStateRunning(State.PARK);
                    break;
                case POWERSHOTS:
                    resetEncoders();
                    useEncoders();
                    break;
                case TOSECONDWOBBLE:
                    if(ringHeight==0){
                        //wobbleClaw.setPosition(.75);
                        resetEncoders();
                        useEncoders();
                        encoderBackwards(5, .7);
                        resetEncoders();
                        useEncoders();
                        encoderCrab(37, .6);
                        turnDegrees(175,.5);
                        resetEncoders();
                        useEncoders();
                        encoderForwards(25, .7);
                        resetEncoders();
                        useEncoders();
                        encoderForwards(5, .4);
                        clawPos=1;
                        wobbleClaw.setPosition(clawPos);
                        sleep(300);
                        runtime.reset();
                        while(runtime.seconds()<1.5){
                            //wobbleClaw.setPosition(1);
                            wobblePivotTop.setPosition(0);
                            wobblePivotBottom.setPosition(1);
                            wobbleThirdPivot.setPosition(0);
                        }
                        setStateRunning(State.DELIVER2);
                    }
                    else if (ringHeight == 1){
                        clawPos=.75;
                        wobbleClaw.setPosition(clawPos);
                        resetEncoders();
                        useEncoders();
                        encoderBackwards(10, .7);
                        //get intake down
                        /*resetEncoders();
                        useEncoders();
                        encoderBackwards(10, .7);
                        resetEncoders();
                        useEncoders();
                        encoderForwards(5, .7);
                        turnDegrees(180,.5);
                        resetEncoders();
                        useEncoders();
                        encoderForwards(30,.7);*/
                        setStateRunning(State.STOP);
                    }
                    break;
                case DELIVER2:
                    telemetry.addData("time", autoTime.seconds());
                    telemetry.update();
                    if(ringHeight==0){
                        telemetry.addData("time", autoTime.seconds());
                        telemetry.update();
                        resetEncoders();
                        useEncoders();
                        encoderBackwards(47, .7);
                        //turnDegrees(95,-.5);
                        //encoderForwards(10, .7);
                        setStateRunning(State.STOP);
                        //get intake down
                        /*resetEncoders();
                        useEncoders();
                        encoderBackwards(10, .65);
                        resetEncoders();
                        useEncoders();
                        encoderForwards(7, .65);
                        resetEncoders();
                        useEncoders();
                        encoderForwards(10, .65);
                        setStateRunning(State.STOP);*/
                    }
                    else if (ringHeight == 1){
                        resetEncoders();
                        useEncoders();
                        encoderBackwards(40, .65);
                        wobblePivotTop.setPosition(1);
                        wobblePivotBottom.setPosition(0);
                        wobbleThirdPivot.setPosition(1);
                        setStateRunning(State.PARK);
                    }
                    break;
                case PARK:
                    if(ringHeight==0)
                    {
                        resetEncoders();
                        useEncoders();
                        encoderForwards(15, .7);
                        turnDegrees(98,.45);
                        resetEncoders();
                        useEncoders();
                        encoderForwards(10, .65);
                        // wobblePivotTop.setPosition(1);
                        // wobblePivotBottom.setPosition(0);
                        clawPos=1;
                        wobbleClaw.setPosition(clawPos);
                    }
                    else if(ringHeight==1)
                    {
                        clawPos=.75;
                        wobbleClaw.setPosition(clawPos);
                        resetEncoders();
                        useEncoders();
                        encoderBackwards(12, .65);
                    }
                    else if(ringHeight==4)
                    {
                        resetEncoders();
                        useEncoders();
                        encoderForwards(3, 0.65);
                    }
                    setStateRunning(State.STOP);
                    break;
                case STOP:
                    shooterWheel.setPower(0);
                    stopMotors();
                    telemetry.addData("time:", autoTime.seconds());
                    telemetry.update();
                    break;
            }
        }
    }

    public void setStateRunning(State s)
    {
        resetEncoders();
        useEncoders();
        stopMotors();
        state = s;
        runtime.reset();
    }

    public void resetEncoders()
    {
        driveFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void useEncoders()
    {
        driveFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        driveBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void encoderForwards(int inches, double power){
        final double WHEEL_DIAMETER = (7.5/2.54);
        final double COUNTS_PER_INCH = 537.6 / (Math.PI * WHEEL_DIAMETER);
        final int STRAIGHT_COUNTS = (int) (COUNTS_PER_INCH*inches);

        while (Math.abs(driveBackRight.getCurrentPosition()) < Math.abs(STRAIGHT_COUNTS)){
            drive(power);
            telemetry.addData("STRAIGHT_COUNTS",STRAIGHT_COUNTS);
            telemetry.addData("POSITION", driveBackRight.getCurrentPosition());
            telemetry.update();
        }

        stopMotors();

        return;
    }

    public void encoderCrab(int inches, double power){
        final double WHEEL_DIAMETER = 7.5; //in cm
        final double COUNTS_PER_CM = 560 / (Math.PI * WHEEL_DIAMETER);
        final int STRAIGHT_COUNTS = (int) (COUNTS_PER_CM*2.54*inches);

        while(Math.abs(driveBackRight.getCurrentPosition()) < Math.abs(STRAIGHT_COUNTS)){
            crab(power);
            telemetry.addData("STRAIGHT_COUNTS",STRAIGHT_COUNTS);
            telemetry.addData("POSITION", driveBackRight.getCurrentPosition());
            telemetry.update();
        }
        stopMotors();
        return;
    }

    public void encoderBackwards(int inches, double power){
        final double WHEEL_DIAMETER = 7.5; //in cm
        final double COUNTS_PER_CM = 560 / (Math.PI * WHEEL_DIAMETER);
        final int STRAIGHT_COUNTS = (int) (COUNTS_PER_CM*2.54*inches*-1);

        while(driveBackRight.getCurrentPosition() > STRAIGHT_COUNTS)
        {
            drive(-power);
            telemetry.addData("STRAIGHT_COUNTS", STRAIGHT_COUNTS);
            telemetry.addData("POSITION", driveBackRight.getCurrentPosition());
            telemetry.update();
        }
        stopMotors();

        return;
    }

    public void drive(double power)
    {
        driveFrontRight.setPower(power);
        driveFrontLeft.setPower(power*.95);
        driveBackRight.setPower(power);
        driveBackLeft.setPower(power*.95);
    }

    public void crab(double power)
    {
        driveFrontRight.setPower(power);
        driveFrontLeft.setPower(-power);
        driveBackRight.setPower(-power);
        driveBackLeft.setPower(power);
    }

    public void turn(double power)
    {
        driveFrontRight.setPower(power);
        driveFrontLeft.setPower(-power);
        driveBackRight.setPower(power);
        driveBackLeft.setPower(-power);
    }

    public void turnTime(double time, double power)
    {
        runtime.reset();
        while(runtime.seconds()<time)
        {
            driveFrontRight.setPower(power);
            driveFrontLeft.setPower(-power);
            driveBackRight.setPower(power);
            driveBackLeft.setPower(-power);
        }
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }
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
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void turnDegrees(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees > 0)
        {   // turn left.
            leftPower = power;
            rightPower = -power;
        }
        else if (degrees < 0)
        {   // turn right.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        driveFrontLeft.setPower(leftPower);
        driveFrontRight.setPower(rightPower);
        driveBackLeft.setPower(leftPower);
        driveBackRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {
                telemetry.addData("angle:", getAngle());
                telemetry.update();
            }
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {
                telemetry.addData("angle:", getAngle());
                telemetry.update();
            }

        // turn the motors off.
        driveFrontLeft.setPower(0);
        driveFrontRight.setPower(0);
        driveBackLeft.setPower(0);
        driveBackRight.setPower(0);

        // wait for rotation to stop.
        sleep(100);

        // reset angle tracking on new heading.
        resetAngle();
        return;
    }
    public void stopMotors()
    {
        driveFrontRight.setPower(0);
        driveFrontLeft.setPower(0);
        driveBackRight.setPower(0);
        driveBackLeft.setPower(0);
    }
    public void flicker()
    {
        shooterFlicker.setPosition(.5);
        runtime.reset();
        while (runtime.seconds()<0.4)
        {
        }
        shooterFlicker.setPosition(.3);
    }

    public void shooterOnly(float seconds,ElapsedTime time){
        while(time.seconds()>seconds){
            driveFrontRight.setPower(0);
            driveFrontLeft.setPower(0);
            driveBackLeft.setPower(0);
            driveBackRight.setPower(0);
        }

    }
    public void encoderShoot(int revs, double power){
        final double WHEEL_DIAMETER = 9; //in cm
        final double COUNTS_PER_CM = 560 / (Math.PI * WHEEL_DIAMETER);
        //final int STRAIGHT_COUNTS = (int) (COUNTS_PER_CM*2.54*inches);

        while(Math.abs(shooterWheel.getCurrentPosition()) < Math.abs(revs)){
            shooterWheel.setPower(power);
            //telemetry.addData("STRAIGHT_COUNTS",STRAIGHT_COUNTS);
            telemetry.addData("POSITION", shooterWheel.getCurrentPosition());
            telemetry.update();
        }
        stopMotors();
        return;
    }
    private void initVuforia() {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
