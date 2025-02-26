package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp

public class troutDrive extends OpMode {
    private DcMotor lfront;
    private DcMotor rfront;
    private DcMotor lback;
    private DcMotor rback;
    private DcMotor firstStage;
    private DcMotor secondStage;
    private Servo claw;

// Sets up PID loops; general naming scheme is firstStage for shoulder, secondStage for elbow
    private PIDController firstStageController;
    public static double p1 = 0, i1 = 0, d1 = 0;


    private PIDController secondStageController;
    public static double p2 = 0, i2 = 0, d2 = 0;


    public static double target1 = 0;
    public static double target2 = 0;
    private int actualValue;
    private int actualValue1;


    // States for state machine
    public enum ARM_STATE {
        ARM_START,
        ARM_TOP_SPECIMEN,
        ARM_TOP_BUCKET,
        ARM_OPEN,

    }

    ARM_STATE armstate = ARM_STATE.ARM_START;


    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Motors and Servos
        lfront = hardwareMap.get(DcMotor.class, "left_front");
        rfront = hardwareMap.get(DcMotor.class, "right_front");
        lback = hardwareMap.get(DcMotor.class, "left_back");
        rback = hardwareMap.get(DcMotor.class, "right_back");
        firstStage = hardwareMap.get(DcMotor.class, "first");
        secondStage = hardwareMap.get(DcMotor.class, "second");
        claw = hardwareMap.get(Servo.class, "claw");

        //Reseting the Motor encoder values. Shouldn't need this
        firstStage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        secondStage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        firstStage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        secondStage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Creating PID loops
        firstStageController = new PIDController(p1,i1,d1);
        secondStageController = new PIDController(p2,i2,d2);

        //Setting 0 power behavior, Motor Direction
        rback.setDirection(DcMotor.Direction.REVERSE);
        rfront.setDirection(DcMotor.Direction.REVERSE);
        lfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        secondStage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        firstStage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        AnalogInput encoder2 = hardwareMap.get(AnalogInput.class, "encoder2");
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "encoder");

        actualValue = (int) ((encoder2.getVoltage() / 3.2 * 360) % 360);
        actualValue1 = (int) ((encoder.getVoltage() / 3.2 * 360) % 360);

        armstate = ARM_STATE.ARM_START;
        p1= 0;
        p2 =0;
        i1=0;
        i2=0;
        d1=0;
        d2=0;


    }
    public void start(){

    }

    public void loop() {
        double rtx = gamepad1.right_stick_x;
        double rty = gamepad1.right_stick_y;
        double ltx = gamepad1.left_stick_x;
        boolean b2 = gamepad2.b;
        double lt2 = gamepad2.left_trigger;
        double rt2 = gamepad2.right_trigger;
        boolean up2 = gamepad2.dpad_up;
        boolean down2 = gamepad2.dpad_down;
        boolean lb2 = gamepad2.left_bumper;
        boolean rb2 = gamepad2.right_bumper;
        boolean triangle2 = gamepad2.y;
        boolean square2 = gamepad2.x;

        // Encoder Stuff

        // encoder 1 (shoulder) (stage 1)
        AnalogInput encoder = hardwareMap.get(AnalogInput.class, "encoder");

        int offset1 =  350; // This changes the initial position of the encoder.
        // To find offset, Subtract the encoder value at start State

        int position1 = (int) ((encoder.getVoltage() / 3.2 * 360 + offset1) % 360);
        telemetry.addData("angle1:", position1);

        // encoder 2 (elbow) (stage 2)
        AnalogInput encoder2 = hardwareMap.get(AnalogInput.class, "encoder2");

        double offset2 = -(actualValue-5) + 360; // This changes the initial position of the encoder.
        // To find offset, Subtract the encoder value at start State

        int position2 = (int) ((encoder2.getVoltage() / 3.2 * 360 + offset2) % 360);
        telemetry.addData("angle2:", position2);




        //PID Stuff 1
        firstStageController.setPID(p1, i1, d1);
        double power1 = firstStageController.calculate(position1, target1);

        firstStage.setPower(power1);
        telemetry.addData("pos1", position1);

        telemetry.addData("target", target1);

        //PID Stuff 2
        secondStageController.setPID(p2, i2, d2);
        double power2 = firstStageController.calculate(position2, target2);
        secondStage.setPower(power2);
        telemetry.addData("pos2", position2);

        telemetry.addData("target2", target2);

// Mechanum Drive Code
        double powerfactor = 1;
        double rotate = -ltx;
        double x = -rtx;
        double y = -rty;
        double fleft = (x + y - rotate) * powerfactor;
        double fright = (-x + y + rotate) * powerfactor;
        double bleft = (-x + y - rotate) * powerfactor;
        double bright = (x + y + rotate) * powerfactor;


        lfront.setPower(fleft);
        rfront.setPower(fright);
        lback.setPower(bleft);
        rback.setPower(bright);

        // Claw code

        if (b2){
            claw.setPosition(0.5);
        } else{
            claw.setPosition(0);
        }



        //Setting PID Values for all states; can be moved


        //State Machine


        // State Arm Open
         int armOpenT1= 353;
         int armOpenT2 = 130;


         // State: Top Specimin

        int topSpecimenT1 = 303;
        int topSpecimenT2 = 84;

        // State: Bucket
        int bucketT1 = 250;
        int bucketT2 = 15;
// Actual Code for State Machine
        switch (armstate) {
            case ARM_START:
                if (square2) {
                    target1 = armOpenT1;
                    target2 = armOpenT2;
                    p2= 0.005;
                    p1= -0;
                    armstate = ARM_STATE.ARM_OPEN;
                }
                if (triangle2) {
                    target1 = armOpenT1;
                    target2 = armOpenT2;
                    p2= 0.005;
                    p1= 0;
                    armstate = ARM_STATE.ARM_OPEN;
                }
                break;


            case ARM_OPEN:
                if (triangle2) {
                   target1 = bucketT1;
                   target2 = bucketT2;
                    p1=-0.005;
                    i1=0;
                    p2=0.005;
                    i2=0;
                   armstate = ARM_STATE.ARM_TOP_BUCKET;
                }
                if (square2) {
                    target1 = topSpecimenT1;
                    target2 = topSpecimenT2;
                    armstate = ARM_STATE.ARM_TOP_SPECIMEN;
                }

                break;

            case ARM_TOP_BUCKET:

            case ARM_TOP_SPECIMEN:
                if (square2 || triangle2) {
                    target1 = armOpenT1;
                    target2 = armOpenT2;
                    p2= 0.005;
                    p1= 0;
                    armstate = ARM_STATE.ARM_OPEN;
                }
                break;


            default:
                armstate = ARM_STATE.ARM_START;

        }
        telemetry.addData("State:", armstate);

        // Manual adjustment for secondStage

        if (up2){
            target2+=0.5;
        }
        if(down2){
            target2 -= 0.5;
        }

    }

}
