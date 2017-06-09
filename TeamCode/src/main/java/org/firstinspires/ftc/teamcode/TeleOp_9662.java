package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="TeleOp", group="#9662")
public class TeleOp_9662 extends LinearOpMode {

    // Declare OpMode members
    Hardware robot = new Hardware();
    StopWatch stopwatch = new StopWatch();

    boolean motorReversed = true;
    boolean ballHolderState = true;
    boolean ropeHolder = true;
    boolean running = false;
    boolean leftBumperPush = false;
    boolean rightBumperPush = false;

    long timeReverse = 0;
    long timeMotorSpeed = 0;
    long timeElevatorPower = 0;
    long timeThrowPower = 0;
    long timeBallHolder = 0;
    long timeBack = 0;
    long timeStart = 0;
    long leftBumperTime = 0;
    long rightBumperTime = 0;
    long timeRPS = 0;

    float ElevatorPower = 0;
    float ThrowPower = 1;
    double Speed = 1;


    @Override
    public void runOpMode() {
        float left;
        float right;
        // Initialize the hardware variables.
        robot.init(hardwareMap);

        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        telemetry.addData("Press play to continue..", "");
        telemetry.update();

        //robot.ColoradoReader.write8(3, robot.LedOFF); //set beacon's sensor's led off for reading light from the beacon

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /** ==== global ==== **/

            /** ==== Driver ==== **/

            // Gamepad1 JoySticks Control
            if (motorReversed) {
                left = -gamepad1.left_stick_y;
                right = -gamepad1.right_stick_y;
                robot.Motor1.setDirection(DcMotor.Direction.FORWARD);
                robot.Motor2.setDirection(DcMotor.Direction.REVERSE);
                robot.Motor3.setDirection(DcMotor.Direction.REVERSE);
                robot.Motor4.setDirection(DcMotor.Direction.FORWARD);
            } else {

                right = -gamepad1.left_stick_y;
                left = -gamepad1.right_stick_y;
                robot.Motor1.setDirection(DcMotor.Direction.REVERSE);
                robot.Motor2.setDirection(DcMotor.Direction.FORWARD);
                robot.Motor3.setDirection(DcMotor.Direction.FORWARD);
                robot.Motor4.setDirection(DcMotor.Direction.REVERSE);
            }

            robot.Motor1.setPower(left * Speed);
            robot.Motor2.setPower(left * Speed);
            robot.Motor3.setPower(right * Speed);
            robot.Motor4.setPower(right * Speed);

            // Gamepad1 B Button Reverse Driving Motors
            if (gamepad1.b && System.currentTimeMillis() - timeReverse > 500) {
                motorReversed = !motorReversed;
                timeReverse = System.currentTimeMillis();
            }

            // Gamepad1 Dpad Up/Down Control Driving Speed
            if (gamepad1.dpad_up && System.currentTimeMillis() - timeMotorSpeed > 200) {
                if (Speed < 1) {
                    Speed += 0.2;
                }
                timeMotorSpeed = System.currentTimeMillis();
            }
            if (gamepad1.dpad_down && System.currentTimeMillis() - timeMotorSpeed > 200) {
                if (Speed > 0.1) {
                    Speed -= 0.2;
                }
                timeMotorSpeed = System.currentTimeMillis();
            }

            // Gamepad1 Left Trigger Control Throwing
            if(gamepad1.right_trigger > 0.8) {
                robot.ThrowRight.setPower(ThrowPower);
                robot.ThrowLeft.setPower(ThrowPower);
                if(!running) {  //set new stopwatch
                    stopwatch.start();
                    running = true;
                }

                if(stopwatch.getElapsedTime() > 800) { //if warm up is over // was  1200
                    //open the stoppers
                    robot.rightStopper.setPosition(1);
                    robot.leftStopper.setPosition(0);
                }
                else {
                    //else stopper is closed
                    robot.rightStopper.setPosition(0);
                    robot.leftStopper.setPosition(1);
                }
            }
            else if(gamepad1.right_bumper) {
                robot.ThrowRight.setPower(-ThrowPower);
                robot.ThrowLeft.setPower(-ThrowPower);
                //open stoppers
                robot.rightStopper.setPosition(1);
                robot.leftStopper.setPosition(0);
            }
            else {
                //stop throw
                robot.ThrowRight.setPower(0);
                robot.ThrowLeft.setPower(0);
                //close the stopper
                robot.rightStopper.setPosition(0);
                robot.leftStopper.setPosition(1);
                stopwatch.stop();
                running = false;
            }

            //Gamepad1 Back\Start Buttons set power to 100 and 0. start - 100 back - 0
            if(gamepad1.back && System.currentTimeMillis() - timeBack > 200) {
                Speed = 0.2;
                timeBack = System.currentTimeMillis();
            }
            if(gamepad1.start && System.currentTimeMillis() - timeStart > 200){
                Speed = 1;
                timeStart = System.currentTimeMillis();
            }

            /** ==== Operator ==== **/

            // Gamepad2 Right Joystick Control Elevator
            if(!ropeHolder) {
                if (gamepad2.right_stick_y < 0) {
                    robot.Elevator.setPower(gamepad2.right_stick_y);
                } else if (gamepad2.right_stick_y > 0) {
                    robot.Elevator.setPower(gamepad2.right_stick_y * 0.05);
                }
                else {
                    robot.Elevator.setPower(0);
                }
            }
            else
            {
                robot.Elevator.setPower(0);
            }

            // Gamepad2 Dpad Up/Down Control Elevator Power
            if(gamepad2.dpad_up && System.currentTimeMillis() - timeElevatorPower > 200) {
                ElevatorPower = 1;
                timeElevatorPower = System.currentTimeMillis();
            }
            if(gamepad2.dpad_down && System.currentTimeMillis() - timeElevatorPower > 200) {
                ElevatorPower = (float)0.05;
                timeElevatorPower = System.currentTimeMillis();
            }

            // Gamepad2 Dpad Left/Right Control Elevator Power
            if(gamepad2.dpad_right && System.currentTimeMillis() - timeElevatorPower > 200) {
                if(ElevatorPower < 1){
                    ElevatorPower += 0.1;
                }
                timeElevatorPower = System.currentTimeMillis();
            }
            if(gamepad2.dpad_left && System.currentTimeMillis() - timeElevatorPower > 200) {
                if(ElevatorPower > 0.1) {
                    ElevatorPower -= 0.1;
                }
                timeElevatorPower = System.currentTimeMillis();
            }

            // Gamepad2 Button Y/A Control Throw Power
            if(gamepad2.y && System.currentTimeMillis() - timeThrowPower > 200) {
                if(ThrowPower < 1) {
                    ThrowPower += 0.05;
                }
                timeThrowPower = System.currentTimeMillis();
            }
            else if(gamepad2.a && System.currentTimeMillis() - timeThrowPower > 200) {
                if(ThrowPower > 0.1) {
                    ThrowPower -= 0.05;
                }
                timeThrowPower = System.currentTimeMillis();
            }

            // Gamepad2 Right Trigger Control Collector
            if(gamepad2.right_trigger > 0.8) {
                robot.Collector.setPower(1);
            }
            else if(gamepad2.right_bumper) {
                robot.Collector.setPower(-1);
            }
            else {
                robot.Collector.setPower(0);
            }

            if(gamepad2.left_stick_y > 0.8){
                ropeHolder = true;
                robot.ropeHolder.setPosition(robot.ROPE_HOLD);
            }
            else if(gamepad2.left_stick_y < -0.8){
                ropeHolder = false;
                robot.ropeHolder.setPosition(robot.ROPE_RELEASE);
            }

            if(gamepad2.x && System.currentTimeMillis() - leftBumperTime > 350){
                leftBumperPush = !leftBumperPush;
                leftBumperTime = System.currentTimeMillis();
            }

            if(leftBumperPush){
                robot.leftBumper.setPosition(robot.LEFT_BEACON_PUSH);
            }
            else{
                robot.leftBumper.setPosition(robot.LEFT_BEACON_PULL);
            }

            if(gamepad2.b && System.currentTimeMillis() - rightBumperTime > 350){
                rightBumperPush = !rightBumperPush;
                leftBumperTime = System.currentTimeMillis();
            }

            if(rightBumperPush){
                robot.rightBumper.setPosition(robot.RIGHT_BEACON_PUSH);
            }
            else{
                robot.rightBumper.setPosition(robot.RIGHT_BEACON_PULL);
            }

            /** ==== telemetry ==== **/

            // Send telemetry message to signify robot running;
            telemetry.addData("Throw Power", "%.1f", ThrowPower * 100);
            telemetry.addData("Elevator Power", "%.1f", ElevatorPower * 100);
            telemetry.addData("\n", "");
            telemetry.addData("Driving Power", "%.1f", Speed * 100);
            telemetry.addData("Motors Reversed? ", motorReversed);
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
        }
    }
}

