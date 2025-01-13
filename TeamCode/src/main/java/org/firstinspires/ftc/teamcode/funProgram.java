package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp

public class funProgram extends OpMode{
    private DcMotor lfront;
    private DcMotor rfront;
    private DcMotor lback;
    private DcMotor rback;
    //private DcMotor shoulder;
   // private DcMotor elbow;
    private Servo claw;

    public void init(){
        lfront = hardwareMap.get(DcMotor.class, "left_front");
        rfront = hardwareMap.get(DcMotor.class, "right_front");
        lback = hardwareMap.get(DcMotor.class, "left_back");
        rback = hardwareMap.get(DcMotor.class, "right_back");
      //  shoulder = hardwareMap.get(DcMotor.class, "shoulder");
       // elbow = hardwareMap.get(DcMotor.class, "elbow");
        claw = hardwareMap.get(Servo.class,  "claw");

        rback.setDirection(DcMotor.Direction.REVERSE);
        rfront.setDirection(DcMotor.Direction.REVERSE);
        lfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //elbow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void loop(){
        double rtx = gamepad1.right_stick_x;
        double rty = gamepad1.right_stick_y;
        double ltx = gamepad1.left_stick_x;
        double lty = gamepad1.left_stick_y;
        boolean circle = gamepad1.circle;
        boolean cross = gamepad1.cross;
        boolean square = gamepad1.square;
        boolean triangle = gamepad1.triangle; //triangle is used for test claw
        boolean lbumper = gamepad1.left_bumper;
        boolean rbumper = gamepad1.right_bumper;
        double lt = gamepad1.left_trigger;
        double rt = gamepad1.right_trigger;
        boolean ltpress = gamepad1.left_stick_button;
        boolean rtpress = gamepad1.right_stick_button;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        double rtx2 = gamepad2.right_stick_x;
        double rty2 = gamepad2.right_stick_y;
        double ltx2 = gamepad2.left_stick_x;
        double lty2 = gamepad2.left_stick_y;
        boolean circle2 = gamepad2.circle;
        boolean cross2 = gamepad2.cross;
        boolean square2 = gamepad2.square;
        boolean triangle2 = gamepad2.triangle;
        boolean lbumper2 = gamepad2.left_bumper;
        boolean rbumper2 = gamepad2.right_bumper;
        double lt2 = gamepad2.left_trigger;
        double rt2 = gamepad2.right_trigger;
        boolean ltpress2 = gamepad2.left_stick_button;
        boolean rtpress2 = gamepad2.right_stick_button;
        boolean up2 = gamepad2.dpad_up;
        boolean down2 = gamepad2.dpad_down;
        boolean left2 = gamepad2.dpad_left;
        boolean right2 = gamepad2.dpad_right;



        double powerfactor = 1;
        double rotate = -ltx;
        double x = -rtx;
        double y = -rty;
        double fleft = (x+y-rotate)*powerfactor;
        double fright = (-x+y+rotate)*powerfactor;
        double bleft = (-x+y-rotate)*powerfactor;
        double bright = (x+y+rotate)*powerfactor;

        if (triangle){
            claw.setPosition(0.5);
        } else {
            claw.setPosition(0);
        }
        lfront.setPower(fleft);
        rfront.setPower(fright);
        lback.setPower(bleft);
        rback.setPower(bright);

        //shoulder.setPower(rty2);
        //elbow.setPower(lty2);





    }

}

