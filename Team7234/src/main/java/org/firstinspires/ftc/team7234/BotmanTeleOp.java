package org.firstinspires.ftc.team7234;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.io.IOException;
import java.util.Formatter;
import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.Logger;

@TeleOp(name="BotmanTeleOp", group="Pushbot")
//@Disabled
public class BotmanTeleOp extends OpMode{

    /* Declare OpMode members. */
    private HardwareBotman robot       = new HardwareBotman();

    //Create Logger
    private final static Logger LOGGER = Logger.getLogger(BotmanTeleOp.class.getName());

    //region Local Variable Declaration

    private static final double driveCurve = 1.0;
    private static final double extensionPow = 0.5;

    private double driveMultiplier = 1.0;
    private double armPower = 0;
    private double relicIncrementing = 0;
    private double relicPos;
    private double headingLock;

    private double targetHead;

    private boolean isMecanum;
    private boolean speedControl;

    private boolean mecanumToggle;
    private boolean gripperToggle;
    private boolean speedToggle;
    private boolean orientationToggle;
    private boolean rotationToggle;

    double magnitude;
    double rotation;
    double angle;

    private HardwareBotman.GripperState gripState = HardwareBotman.GripperState.OPEN;

    private enum turningState{
        NORMAL,
        LEFT,
        RIGHT,
        LOCKED
    }

    private turningState turnState = turningState.NORMAL;

    //endregion

    @Override
    public void init() {
        robot.init(hardwareMap, false, DcMotor.ZeroPowerBehavior.BRAKE);

        try {
            FileHandler handler = new FileHandler("botmanlog%g.html");
            handler.setFormatter(new BotmanHtmlFormatter());
            LOGGER.addHandler(handler);
        }
        catch (IOException e){
            e.printStackTrace();
            LOGGER.severe("Problems Creating Files");
        }


        //region Boolean Initialization

        //Controlling Booleans
        isMecanum = true;
        speedControl = false;

        //Toggle Booleans
        mecanumToggle = true;
        gripperToggle = true;
        speedToggle = true;
        orientationToggle = true;
        rotationToggle = true;

        //endregion
        relicPos = robot.relicClaw.getPosition();

        LOGGER.setLevel(Level.INFO);
        LOGGER.info("Initializaton Completed");
    }
    @Override
    public void init_loop(){}
    @Override
    public void start(){}
    @Override
    public void loop() {
        //region Values
        driveMultiplier = speedControl ? 0.5 : 1;

        //calculates angle in radians based on joystick position, reports in range [-Pi/2, 3Pi/2]
        angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + (Math.PI / 2);
        if (Double.isNaN(angle)){
            angle = 0;              //Prevents NaN error later in the Program
        }
        //calculates robot speed from the joystick's distance from the center
        magnitude = driveMultiplier*Math.pow(Range.clip(Math.sqrt(Math.pow(gamepad1.left_stick_x, 2) + Math.pow(gamepad1.left_stick_y, 2)), 0, 1), driveCurve);

        // How much the robot should turn while moving in that direction

        switch(turnState){
            case NORMAL:
                rotation = driveMultiplier* Range.clip(gamepad1.right_stick_x, -1, 1);
                break;
            case LEFT:
                if (robot.heading() > targetHead - 3.0 && robot.heading() < targetHead + 3.0){
                    turnState = turningState.NORMAL;
                    LOGGER.info("Left Turn Complete, Target heading was: "
                            + targetHead
                            + "\nEnd heading was: "
                            + robot.heading()
                    );
                    break;
                }
                else{
                    rotation = -0.5;
                    break;
                }
            case RIGHT:
                if (robot.heading() >targetHead - 3.0 && robot.heading() < targetHead + 3.0){
                    turnState = turningState.NORMAL;
                    LOGGER.info("Right Turn Complete, Target heading was: "
                            + targetHead
                            + "\nEnd heading was: "
                            + robot.heading()
                    );
                    break;
                }
                else{
                    rotation = 0.5;
                    break;
                }
            case LOCKED:
                if (robot.heading() < headingLock){
                    rotation = 0.1;
                }
                else if (robot.heading() > headingLock){
                    rotation = -0.1;
                }
                break;
        }

        //Variables for tank drive
        double left = -gamepad1.left_stick_y;
        double right = -gamepad1.right_stick_y;

        double relicPower = (gamepad2.x) ? Range.clip(gamepad2.left_stick_y, -1.0, 1) : 0;

        if (robot.armLimit.getState() && armPower < 0){
            armPower = 0;
        }
        else {
            armPower = gamepad2.left_trigger - gamepad2.right_trigger;
        }
        //endregion
        //region 5 Claw
        relicIncrementing = gamepad2.right_stick_y / 500.0;

        if (relicPos + relicIncrementing > 1.0){
            relicPos = 1.0;
        }
        else if (relicPos + relicIncrementing < 0.0){
            relicPos = 0.0;
        }
        else{
            relicPos += relicIncrementing;
        }
        robot.relicClaw.setPosition(relicPos);
        //endregion
        //region Toggles

        //region Mecanum Toggles
        if (mecanumToggle){ //Toggles drive mode based on the x button
            if (gamepad1.x){
                isMecanum = !isMecanum;
                mecanumToggle = false;
            }
        }
        else if (!gamepad1.x) {
            mecanumToggle = true;
        }
        //endregion

        //region Gripper Toggle
        //cycles through gripper states, once for each button press
        if (gripperToggle){
            if (gamepad2.a){
                gripState = gripState.next();
                gripperToggle = false;
                LOGGER.info("Gripper State changed, state is now: " + gripState);
            }
            if (gamepad2.b){
                gripState = gripState.previous();
                gripperToggle = false;
                LOGGER.info("Gripper State changed, state is now: " + gripState);
            }
        }
        else if (!(gamepad2.a || gamepad2.b) ){
            gripperToggle = true;
        }
        //endregion

        //region Speed Toggle
        if (speedToggle){
            if(gamepad1.b){
                speedControl = !speedControl;
                LOGGER.info("Speed Scale altered, Scale is now: " + driveMultiplier);
                speedToggle = false;
            }
        }
        else if (!gamepad1.b){
            speedToggle = true;
        }
        //endregion

        //region Orientation Lock
        if (orientationToggle){
            if (gamepad1.right_stick_button){
                if (turnState == turningState.LOCKED){
                    turnState = turningState.NORMAL;
                }
                else {
                    turnState = turningState.LOCKED;
                }
                headingLock = robot.heading();
                LOGGER.info("Orientation Lock Changed, state is now:"
                        + turnState
                        + "\nTarget heading is: "
                        + headingLock
                );
                orientationToggle = false;
            }
        }
        else if (!gamepad1.right_stick_button){
            orientationToggle = true;
        }
        //endregion

        //region Rotations
        if (rotationToggle){
            if (gamepad1.dpad_left){
                turnState = turningState.LEFT;
                targetHead = (robot.heading() > 90.0) ? robot.heading() -270.0 : robot.heading() +90;
                rotationToggle = false;
                LOGGER.info("Now turning Left, current heading is: "
                        + robot.heading()
                        + "\nTarget heading is: "
                        + targetHead
                );
            }
            else if (gamepad1.dpad_right){
                turnState = turningState.RIGHT;
                targetHead = (robot.heading()>-90.0) ? robot.heading() -90 : robot.heading() + 270.0;
                rotationToggle = false;
                LOGGER.info("Now turning Right, current heading is: "
                        + robot.heading()
                        + "\nTarget heading is: "
                        + targetHead
                );
            }
        }
        else if(!gamepad1.dpad_right && !gamepad1.dpad_left){
            rotationToggle = true;
        }
        //endregion

        //endregion
        //region Robot Control

        //Sends Power to the Robot Arm
        robot.arm.setPower(armPower);

        //Drives the robot
        if (isMecanum){
            robot.mecanumDrive(angle, magnitude, rotation); //Drives Omnidirectionally
        }
        else{
            if(!gamepad1.right_bumper && !gamepad1.left_bumper){ //Drives as tank
                robot.arrayDrive(left, left, right, right);
            }
            else if (gamepad1.right_bumper && !gamepad1.left_bumper) {  //Strafe right
                robot.arrayDrive(-1, 1, -1, 1);
            }
            else if (!gamepad1.right_bumper) { //Strafe Left
                robot.arrayDrive(1, -1, 1, -1);
            }
            else{
                robot.arrayDrive(0, 0, 0, 0); //Stop
            }
        }

        //endregion
        //region Gripper Control
        robot.gripperSet(gripState);
        //endregion
        //region Relic Control
        robot.relicArm.setPower(relicPower);
        //endregion
        //region Telemetry

        telemetry.addData("isMecanum: ", isMecanum);
        telemetry.addData("gripperState: ", gripState);
        telemetry.addData("Speed Limited to: ", driveMultiplier);
        telemetry.addData("TurningState: ", turnState);
        telemetry.addLine();
        telemetry.addData("Angle: ", angle);
        telemetry.addData("Magnitude: ", magnitude);
        telemetry.addData("Rotation: ", rotation);
        telemetry.addLine();
        telemetry.addData("Relic Power: ", relicPower);
        telemetry.addData("Relic Position: ", relicPos);
        telemetry.addLine();
        telemetry.addData("X: ", gamepad1.left_stick_x);
        telemetry.addData("Y: ", gamepad1.left_stick_y);
        telemetry.addLine();
        telemetry.addData("FL: ", robot.mecanumSpeeds[0]);
        telemetry.addData("FR: ", robot.mecanumSpeeds[1]);
        telemetry.addData("BR: ", robot.mecanumSpeeds[2]);
        telemetry.addData("BL: ", robot.mecanumSpeeds[3]);
        telemetry.addLine();
        telemetry.addData("FL-pow: ", robot.driveMotors[0].getPower());
        telemetry.addData("FR-pow: ", robot.driveMotors[1].getPower());
        telemetry.addData("BR-pow: ", robot.driveMotors[2].getPower());
        telemetry.addData("BL-pow: ", robot.driveMotors[3].getPower());

        //endregion
    }
    @Override
    public void stop(){}
}
