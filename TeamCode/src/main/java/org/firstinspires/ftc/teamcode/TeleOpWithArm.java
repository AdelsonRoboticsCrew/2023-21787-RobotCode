/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Tele-Op w/ Arm", group="Iterative OpMode")
//@Disabled
public class TeleOpWithArm extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private DcMotor arm = null;
    private DcMotor armExtender = null;
    private Servo claw = null;
    private Servo wrist = null;
    private boolean clawOpen = true;

    private boolean wristDown = true;
    private boolean lastB = false;
    private boolean lastX = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        arm = hardwareMap.get(DcMotor.class, "arm");
        armExtender = hardwareMap.get(DcMotor.class, "arm_extender");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        arm.setDirection(DcMotor.Direction.FORWARD);
        armExtender.setDirection(DcMotor.Direction.FORWARD);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armExtender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set to default Positions
        arm.setTargetPosition(0);
        armExtender.setTargetPosition(0);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armExtender.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftFrontPower;
        double leftBackPower;
        double rightFrontPower;
        double rightBackPower;


        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        double strafe = gamepad1.left_stick_x;
        leftFrontPower = drive + turn + strafe;
        leftBackPower = drive + turn - strafe;
        rightFrontPower = drive - turn - strafe;
        rightBackPower = drive - turn + strafe;


        double divisor = Math.max(Math.max(leftFrontPower, leftBackPower), Math.max(rightFrontPower, rightBackPower));
        if (divisor >= 0.7) {
            leftFrontPower /= divisor;
            leftBackPower /= divisor;
            rightFrontPower /= divisor;
            rightBackPower /= divisor;
        }
        leftBackPower *= 0.7;
        leftFrontPower *= 0.7;
        rightBackPower *= 0.7;
        rightFrontPower *= 0.7;
        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightFrontDrive.setPower(rightFrontPower);
        rightBackDrive.setPower(rightBackPower);


        if (gamepad2.x && !lastX) {
            clawOpen = !clawOpen;
        }
        if (gamepad2.b && !lastB) {
            wristDown = !wristDown;
        }

        if(clawOpen) {
            telemetry.addLine("claw debug open");
            claw.setPosition(0.5); //find value during testing
        }else{
            telemetry.addLine("claw debug close");
            claw.setPosition(1.0); //find value during testing
        }
/*
        if(wristDown){
            telemetry.addLine("wrist debug down");
            wrist.setPosition(0.8);
        }else{
            telemetry.addLine("wrist debug up");
            wrist.setPosition(0.0);
        }
*/
        int increment = 2;
        if(Range.clip(-gamepad2.left_stick_y, 0.0, 1.0) > 0.3){
            armExtender.setPower(0.4);
            //3620 limit
            if(armExtender.getTargetPosition() + increment < 3620) {
                armExtender.setTargetPosition(armExtender.getTargetPosition() + increment);
            }
            telemetry.addLine("arm go out value: " + armExtender.getTargetPosition());
        }else if(Range.clip(-gamepad2.left_stick_y, -1.0, 0.0) < 0.3){
            armExtender.setPower(0.4);
            //ok so i changed their speed a little so that they have some more control over it
            //IMPORTANT!!! FIND A LIMIT NUMBER FOR EXTENDER!!! MAKE SURE IT DOESN'T BREAK!!!
            if(armExtender.getTargetPosition() - increment > 0) {
                armExtender.setTargetPosition(armExtender.getTargetPosition() - increment);
            }
            telemetry.addLine("arm go in value: " + armExtender.getTargetPosition());
        }

        if(Range.clip(-gamepad2.right_stick_y, 0.0, 1.0) > 0.3){
            arm.setPower(0.2);
            //IMPORTANT!!! FIND A LIMIT NUMBER FOR ARM!!! MAKE SURE IT DOESN'T BREAK!!!
            arm.setTargetPosition(arm.getTargetPosition() + increment);
            telemetry.addLine("arm go up value: " + arm.getTargetPosition());
        }else if(Range.clip(-gamepad2.right_stick_y, -1.0, 0.0) < -0.3){
            arm.setPower(0.2);
            //IMPORTANT!!! FIND A LIMIT NUMBER FOR ARM!!! MAKE SURE IT DOESN'T BREAK!!!
            arm.setTargetPosition(arm.getTargetPosition() - increment);
            telemetry.addLine("arm go down value: " + arm.getTargetPosition());
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left front (%.2f), left back (%.2f), right front (%.2f), right back (%.2f)", leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);

        lastX = gamepad2.x;
        lastB = gamepad2.b;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */


    @Override
    public void stop(){
        
    }
}
