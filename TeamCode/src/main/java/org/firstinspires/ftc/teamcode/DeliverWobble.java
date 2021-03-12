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

@Autonomous(name="Shoot Auto", group="Linear Opmode")

public class DeliverWobble extends LinearOpMode {
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
    private Servo wobbleClaw;

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

    private enum State {
        TOSCAN,
        SCAN,
        TOLL,
        SHOOT1,
        SHOOT2,
        SHOOT3,
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
        //open trapdoor on initalization

        //shooter
        shooterWheel = hardwareMap.get(DcMotor.class, "shooterWheel");
        shooterFlicker = hardwareMap.get(Servo.class, "shooterFlicker");
        shooterFlicker.setPosition(flickerPos);

        //intake
        intake = hardwareMap.get(DcMotor.class, "intake");

        //wobble goal
        wobbleLead = hardwareMap.get(DcMotor.class, "wobbleLead");
        wobblePivotTop = hardwareMap.get(Servo.class, "wobblePivotTop");
        wobblePivotBottom = hardwareMap.get(Servo.class, "wobblePivotBottom");
        wobbleClaw = hardwareMap.get(Servo.class, "wobbleClaw");

        //ring detection
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(2.5, 16.0/9.0);
        }

        driveFrontRight.setDirection(DcMotor.Direction.REVERSE);
        driveBackRight.setDirection(DcMotor.Direction.REVERSE);

        driveFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        resetEncoders();
        useEncoders();
        telemetry.addData("pos", driveFrontLeft.getCurrentPosition());
        telemetry.update();
        waitForStart();
        resetEncoders();
        useEncoders();
        state = State.TOSCAN;
        runtime.reset();
        ElapsedTime autoTime = new ElapsedTime();
        while (opModeIsActive())
        {
            switch(state) {
                case TOSCAN:
                    resetEncoders();
                    useEncoders();
                    encoderForwards(10, .5);
                    setStateRunning(State.SCAN);
                    break;
                case SCAN:
                    if (tfod != null) {
                        List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                        if (updatedRecognitions != null) {
                            telemetry.addData("# Object Detected", updatedRecognitions.size());
                            //int i = 0;
                            for (Recognition r : updatedRecognitions) {
                                if(r.getLabel().equals(LABEL_FIRST_ELEMENT))
                                {
                                    ringHeight = 4;
                                }
                                else if(r.getLabel().equals(LABEL_SECOND_ELEMENT))
                                {
                                    ringHeight = 1;
                                }
                                else
                                {
                                    ringHeight = 0;
                                }
                                /*telemetry.addData(String.format("label (%d)", i), r.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        r.getLeft(), r.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        r.getRight(), r.getBottom());*/
                            }
                            telemetry.addData("# of rings", ringHeight);
                            telemetry.update();
                        }
                    }
                    setStateRunning(State.TOLL);
                    break;
                case TOLL:
                    shooterFlicker.setPosition(0);
                    shooterWheel.setPower(-.57);
                    resetEncoders();
                    useEncoders();
                    if (ringHeight == 4)
                    {
                        encoderForwards(30,.5);
                    }
                    else if (ringHeight == 1)
                    {
                        encoderForwards(20,.5);
                    }
                    else
                    {
                        encoderForwards(10,.5);
                    }
                    /*encoderCrab(5, .35);
                    resetEncoders();
                    useEncoders();
                    encoderForwards(54, .35);
                    //turn(17);
                    resetEncoders();
                    useEncoders();
                    encoderCrab(5, .35);*/
                    setStateRunning(State.STOP);
                    break;
                case SHOOT1:
                    shooterWheel.setPower(-.57);
                    flicker();
                    setStateRunning(State.SHOOT2);
                    break;
                case SHOOT2:
                    shooterFlicker.setPosition(0);
                    shooterWheel.setPower(-.57);
                    encoderCrab(7, .35);
                    flicker();
                    setStateRunning(State.SHOOT3);
                case SHOOT3:
                    shooterFlicker.setPosition(0);
                    shooterWheel.setPower(-.57);
                    encoderCrab(5, .35);
                    flicker();
                    setStateRunning(State.PARK);
                case PARK:
                    resetEncoders();
                    useEncoders();
                    encoderForwards(5, .5);
                    setStateRunning(State.STOP);
                    break;
                case STOP:
                    stopMotors();
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
        driveFrontLeft.setPower(power);
        driveBackRight.setPower(power);
        driveBackLeft.setPower(power);
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

    public void stopMotors()
    {
        driveFrontRight.setPower(0);
        driveFrontLeft.setPower(0);
        driveBackRight.setPower(0);
        driveBackLeft.setPower(0);
    }
    public void flicker()
    {
        shooterFlicker.setPosition(1);
        runtime.reset();
        while (runtime.seconds()<0.8)
        {
        }
        shooterFlicker.setPosition(0);
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

