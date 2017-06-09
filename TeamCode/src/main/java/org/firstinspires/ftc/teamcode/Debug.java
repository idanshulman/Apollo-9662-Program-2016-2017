package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Debug", group="AUTONOMUS")
//@Disabled
public class Debug extends LinearOpMode {

    /* Declare OpMode members. */
    Hardware robot = new Hardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    StopWatch stopwatch = new StopWatch();

    @Override
    public void runOpMode(){

        // Initialize the drive system variables.
        robot.init(hardwareMap);

        // ColorabiL - Line
        // Colorado - Beacon
        robot.ColoradoReader.write8(3, robot.LedOFF); //set beacon's sensor's led off for reading light from the beacon
        robot.ColorabiLReader.write8(3, robot.LedON); //set line's sensor's led on for reading light from the ground.

        //idle();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        /**==== Start Autonomous Operation ====**/

        while(opModeIsActive()){
            telemetry.addData("Sensors: ", "");
            telemetry.addData("Color Beacon Sensor: ", robot.ColoradoReader.read8(4));
            telemetry.addData("Left Line Sensor: ", robot.ColorabiLReader.read8(8));
            telemetry.addData("Right Line Sensor: ", robot.ColorabiRReader.read8(8));
            telemetry.addData("UltiR: ", robot.ultiR.getUltrasonicLevel());
            telemetry.addData("UltiL: ", robot.ultiL.getUltrasonicLevel());

            telemetry.addData("Positions:", "");
            telemetry.addData("Left Position: ", robot.Motor1.getCurrentPosition());
            telemetry.addData("Right Position: ", robot.Motor3.getCurrentPosition());
            telemetry.update();
        }

        /**==== End Autonomous Operation ====**/
    }

    /**==== Functions Area ====**/

    /**==== End Functions Area ====**/
}
