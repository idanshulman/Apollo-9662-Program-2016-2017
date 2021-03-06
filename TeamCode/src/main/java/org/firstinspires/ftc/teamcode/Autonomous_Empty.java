package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.Timer;
import java.util.TimerTask;


@Autonomous(name="Autonomus_Empty", group="AUTONOMUS")
@Disabled
public class Autonomous_Empty extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    StopWatch stopwatch = new StopWatch();

    Timer ShutDown = new Timer();

    static final double COUNTS_PER_MOTOR_REV = 1100;
    static final double DRIVE_GEAR_REDUCTION = 1;
    static final double WHEEL_DIAMETER_CM = 10.0;
    static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);
    static final double WHEEL_DISTANCE_CM = 34;
    static final int COLOR_BLUE = 3;
    static final int COLOR_RED = 10;
    static final int LED_ON = 0;
    static final int LED_OFF = 1;

    boolean isFinishedTurning = true;

    @Override
    public void runOpMode(){
        // Initialize the drive system variables.

        robot.init(hardwareMap);

        // Colorado - Beacon && ColorabiL - Line
        robot.ColoradoReader.write8(3, LED_OFF); //set beacon's sensor's led off for reading light from the beacon
        robot.ColorabiLReader.write8(3, LED_ON); //set line's sensor's led on for reading light from the ground.

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Status", "Resetting Encoders and Calibrating Gyro");    //
        telemetry.update();

        telemetry.addData("Press play to continue..", "");
        telemetry.update();

        idle();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        ShutDown.schedule(new TimerTask() {
            @Override
            public void run() {
                requestOpModeStop();
            }
        }, 29900);

        runtime.reset();

        /**==== Start Autonomous Operation ====**/

        while(opModeIsActive()) {
            Telemetry();
        }

        /**==== End Autonomous Operation ====**/

        // Send telemetry message to indicate end of operation
        telemetry.addData("Autonomus Operation", "Complete");
        telemetry.update();
    }

    /**==== Functions Area ====**/

    public void drive(double cm, double power) {
        int newTarget;
        double distance;

        setEncoderMode(true);

        if(cm > 0) {
            setDirection(true);
        }
        else {
            setDirection(false);
        }
        distance = Math.abs(cm);

        robot.Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        sleep(300);

        // Determine new target position, and pass to motor controller
        newTarget = (int) (distance * COUNTS_PER_CM);
        robot.Motor1.setTargetPosition(newTarget);
        robot.Motor3.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        robot.Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        robot.Motor1.setPower(Math.abs(power));
        robot.Motor2.setPower(Math.abs(power));
        robot.Motor3.setPower(Math.abs(power));
        robot.Motor4.setPower(Math.abs(power));

        // keep looping while we are still active, and there is time left, and both motors are running.
        while (opModeIsActive() && robot.Motor1.getCurrentPosition() < newTarget && robot.Motor3.getCurrentPosition() < newTarget) {
            // Display it for the driver.
            Telemetry();
        }
        // Stop all motion;
        stopMotors();

        // Turn off RUN_TO_POSITION
        robot.Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.Motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);   // optional pause after each move
    }

    public void turn(double angle, double power) {
        //Positive for right
        double newTargetLeft;
        double newTargetRight;

        isFinishedTurning = false;

        resetEncoders();

        if(angle > 0) {
            setDirection(true);
        }
        else {
            setDirection(false);
        }
        angle = Math.abs(angle);

        Sleep(300);

        newTargetLeft = (WHEEL_DISTANCE_CM / WHEEL_DIAMETER_CM) * angle * 3.0;
        newTargetRight = -(WHEEL_DISTANCE_CM / WHEEL_DIAMETER_CM) * angle * 3.0;

        robot.Motor1.setTargetPosition((int)newTargetLeft);
        robot.Motor3.setTargetPosition((int)newTargetRight);

        robot.Motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Motor3.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (opModeIsActive() && robot.Motor1.getCurrentPosition() < newTargetLeft && robot.Motor3.getCurrentPosition() > newTargetRight) {
            Telemetry();
            robot.Motor1.setPower(power);
            robot.Motor2.setPower(power);
            robot.Motor3.setPower(-power);
            robot.Motor4.setPower(-power);
        }

        stopMotors();

        setEncoderMode(true);

        isFinishedTurning = true;
    }

    public void driveToLine(double power) {
        setDirection(true);
        setEncoderMode(false);

        robot.Motor1.setPower(power);
        robot.Motor2.setPower(power);
        robot.Motor3.setPower(power);
        robot.Motor4.setPower(power);

        while (opModeIsActive() && !seeLine()) {
            telemetry.addData("SeeTheLine: ", seeLine());
            Telemetry();
        }
        stopMotors();
    }

    public void throwBall(double power) {
        setStopper(false); //close the stopper.

        //start running the throwing motors.
        robot.ThrowRight.setPower(power);
        robot.ThrowLeft.setPower(power);

        Sleep(800);

        setStopper(true); //open the stopper.

        robot.Collector.setPower(1); //start the collector motor

        Sleep(800);

        //stop running the throwing motors.
        robot.ThrowRight.setPower(0);
        robot.ThrowLeft.setPower(0);
    }

    public void driveTillDistance(int distance, double power){
        //at 0.1 speed error of 2 cm
        //get a reading.
        double dis = robot.ultiR.getUltrasonicLevel();
        setEncoderMode(false);
        setDirection(true);

        while(opModeIsActive() && dis > distance){
            dis = robot.ultiR.getUltrasonicLevel();
            robot.Motor1.setPower(power);
            robot.Motor2.setPower(power);
            robot.Motor3.setPower(power);
            robot.Motor4.setPower(power);
            Telemetry();
        }

        stopMotors();
    }

    public void Telemetry() {
        telemetry.addData("Sensors: ", "");
        telemetry.addData("Color Beacon Sensor: ", robot.ColoradoReader.read8(4));
        telemetry.addData("Left Line Sensor: ", robot.ColorabiLReader.read8(4));
        telemetry.addData("Right Line Sensor: ", robot.ColorabiRReader.read8(4));

        telemetry.addData("Positions:", "");
        telemetry.addData("Left Position: ", robot.Motor1.getCurrentPosition());
        telemetry.addData("Right Position: ", robot.Motor3.getCurrentPosition());
        telemetry.update();
    }

    public void setDirection(boolean front) {
        if(front){
            robot.Motor1.setDirection(DcMotor.Direction.FORWARD);
            robot.Motor2.setDirection(DcMotor.Direction.REVERSE);
            robot.Motor3.setDirection(DcMotor.Direction.REVERSE);
            robot.Motor4.setDirection(DcMotor.Direction.FORWARD);
        }
        else{
            robot.Motor1.setDirection(DcMotor.Direction.REVERSE);
            robot.Motor2.setDirection(DcMotor.Direction.FORWARD);
            robot.Motor3.setDirection(DcMotor.Direction.FORWARD);
            robot.Motor4.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    public void setEncoderMode(boolean runUsingEncoders) {
        if(runUsingEncoders) {
            robot.Motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.Motor3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        else{
            robot.Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.Motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setStopper(boolean state) {
        if(state) {//state indicates the wanted state for the stoppers true for open
            robot.leftStopper.setPosition(0);
            robot.rightStopper.setPosition(1);
        }
        else {//false for close.
            robot.leftStopper.setPosition(1);
            robot.rightStopper.setPosition(0);
        }
    }

    public void resetEncoders() {
        robot.Motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.Motor3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void stopMotors() {
        robot.Motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.Motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.Motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.Motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.Motor1.setPower(0);
        robot.Motor2.setPower(0);
        robot.Motor3.setPower(0);
        robot.Motor4.setPower(0);
    }

    public boolean seeLine() {
        boolean RetVal = false;
        if (robot.ColorabiLReader.read8(4) == 16) {
            RetVal = true;
        }
        if (robot.ColorabiLReader.read8(4) == 6) {
            RetVal = true;
        }
        if (robot.ColorabiLReader.read8(8) > 30) {
            RetVal = true;
        }
        return RetVal;
    }

    public void Sleep(long ms) {
        long time = System.currentTimeMillis();

        while(opModeIsActive() && (System.currentTimeMillis() - time < ms)){
            Telemetry();
            idle();
        }
    }

    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor)
        {
            double voltage = sensor.getVoltage();
            if (voltage > 0)
            {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    /**==== End Functions Area ====**/
}
