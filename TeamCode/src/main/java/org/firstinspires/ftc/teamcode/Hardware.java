package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Hardware
{
    /* Public OpMode members. */

    /** motors declare **/
    public DcMotor Motor1 = null;
    public DcMotor Motor2 = null;
    public DcMotor Motor3 = null;
    public DcMotor Motor4 = null;
    public DcMotor Elevator = null;
    public DcMotor Collector = null;
    public DcMotor ThrowLeft = null;
    public DcMotor ThrowRight = null;

    /** servo declare **/
    public Servo rightStopper = null;
    public Servo leftStopper = null;
    public Servo ropeHolder = null;
    public Servo leftBumper = null;
    public Servo rightBumper = null;

    /** Sensors and I2C Devices declare **/
    I2cAddr ColoradoAddr = I2cAddr.create8bit(0x3c);
    I2cAddr ColorabiRAddr =  I2cAddr.create8bit(0x10);
    I2cAddr ColorabiLAddr = I2cAddr.create8bit(0x20);

    //beacon sensor
    public I2cDevice Colorado = null;
    public I2cDeviceSynch ColoradoReader = null;
    //left line sensor
    public I2cDevice ColorabiL = null;
    public I2cDeviceSynch ColorabiLReader = null;
    //right line sensor
    public I2cDevice ColorabiR = null;
    public I2cDeviceSynch ColorabiRReader = null;

    //lego ultrasonic sensor
    public UltrasonicSensor ultiR = null;
    public UltrasonicSensor ultiL = null;

    //device interface
    public DeviceInterfaceModule Dim;

    public final int LedOFF = 1;
    public final int LedON = 0;

    public static final double COUNTS_PER_MOTOR_REV = 1100;
    public static final double DRIVE_GEAR_REDUCTION = 1;
    public static final double WHEEL_DIAMETER_CM = 10.0;
    public static final double COUNTS_PER_CM = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_CM * 3.1415);
    public static final double WHEEL_DISTANCE_CM = 34;
    public static final int COLOR_BLUE = 3;
    public static final int COLOR_RED = 10;

    public final double ROPE_HOLD = 1;
    public final double ROPE_RELEASE = 0;

    public final double LEFT_BEACON_PULL = 0.45;
    public final double LEFT_BEACON_PUSH = 1;

    public final double RIGHT_BEACON_PULL = 0.62;
    public final double RIGHT_BEACON_PUSH = 0;

    /* Local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public Hardware() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        // driving motors
        Motor1 = hwMap.dcMotor.get("motor1");
        Motor2 = hwMap.dcMotor.get("motor2");
        Motor3 = hwMap.dcMotor.get("motor3");
        Motor4 = hwMap.dcMotor.get("motor4");
        // additional motors
        Elevator = hwMap.dcMotor.get("elevator");
        Collector = hwMap.dcMotor.get("collector");
        ThrowLeft = hwMap.dcMotor.get("throwLeft");
        ThrowRight= hwMap.dcMotor.get("throwRight");

        // Define and Initialize Servos
        rightStopper = hwMap.servo.get("rightStopper");
        leftStopper = hwMap.servo.get("leftStopper");
        ropeHolder  = hwMap.servo.get("ropeHolder");
        leftBumper = hwMap.servo.get("leftBumper");
        rightBumper = hwMap.servo.get("rightBumper");

        // Define and Initialize Sensors, I2C Devices and their Readers.
        //lego ultrasonic sensor
        ultiR = hwMap.ultrasonicSensor.get("ultiR");
        ultiL = hwMap.ultrasonicSensor.get("ultiL");
        //color sensors
        Colorado = hwMap.i2cDevice.get("colorado"); //beacon sensor
        ColorabiL = hwMap.i2cDevice.get("colorabiL"); //left line sensor
        ColorabiR = hwMap.i2cDevice.get("colorabiR"); //right line sensor

        //device interface module
        Dim = hwMap.deviceInterfaceModule.get("Dim");
        //beacon sensor
        ColoradoReader = new I2cDeviceSynchImpl(Colorado, ColoradoAddr, false);
        ColoradoReader.engage();
        //left line sensor
        ColorabiLReader = new I2cDeviceSynchImpl(ColorabiL, ColorabiLAddr, false);
        ColorabiLReader.engage();
        //right line sensor
        ColorabiRReader = new I2cDeviceSynchImpl(ColorabiR, ColorabiRAddr, false);
        ColorabiRReader.engage();


        // Set directions for all motors
        Motor1.setDirection(DcMotor.Direction.FORWARD);
        Motor2.setDirection(DcMotor.Direction.REVERSE);
        Motor3.setDirection(DcMotor.Direction.REVERSE);
        Motor4.setDirection(DcMotor.Direction.FORWARD);
        ThrowLeft.setDirection(DcMotor.Direction.REVERSE);
        ThrowRight.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        Motor1.setPower(0);
        Motor2.setPower(0);
        Motor3.setPower(0);
        Motor4.setPower(0);
        Elevator.setPower(0);

        // Set all motors to run without encoders.
        Motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Motor4.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Elevator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Collector.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ThrowLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ThrowRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set all servos to start position
        rightStopper.setPosition(0);
        leftStopper.setPosition(1);
        ropeHolder.setPosition(ROPE_HOLD);
        leftBumper.setPosition(LEFT_BEACON_PULL);
        rightBumper.setPosition(RIGHT_BEACON_PULL);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     */
    public void waitForTick(long periodMs) {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0) {
            try {
                Thread.sleep(remaining);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Reset the cycle clock for the next pass.
        period.reset();
    }

}