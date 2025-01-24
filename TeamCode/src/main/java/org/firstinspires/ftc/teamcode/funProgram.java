package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
@Config

public class funProgram extends OpMode{
    private DcMotor lfront;
    private DcMotor rfront;
    private DcMotor lback;
    private DcMotor rback;
    private DcMotor firstStage;
    private DcMotor secondStage;
    private Servo claw;
    private PIDController firstStageController;
    private PIDController secondStageController;
    public static double p1=0, i1=0, d1=0;
    public static double f1 = 0;
    public static double p2=0, i2=0, d2=0;
    public static double f2 = 0;

    public static int target1 = 0;
    public static int target2 =0;
    private final double ticks_in_degree = 1.6306513409961;

    public void init(){
        firstStageController = new PIDController(p1,i1,d1);
        secondStageController = new PIDController (p2, i2,d2);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lfront = hardwareMap.get(DcMotor.class, "left_front");
        rfront = hardwareMap.get(DcMotor.class, "right_front");
        lback = hardwareMap.get(DcMotor.class, "left_back");
        rback = hardwareMap.get(DcMotor.class, "right_back");
        firstStage = hardwareMap.get(DcMotor.class, "shoulder");
       secondStage = hardwareMap.get(DcMotor.class, "elbow");
        claw = hardwareMap.get(Servo.class,  "claw");
        secondStage.setPower(0.25);
        
        secondStage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        firstStage.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rback.setDirection(DcMotor.Direction.REVERSE);
        rfront.setDirection(DcMotor.Direction.REVERSE);
        lfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        firstStage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        secondStage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);





    }
    public void loop(){
        double rtx = gamepad1.right_stick_x;
        double rty = gamepad1.right_stick_y;
        double ltx = gamepad1.left_stick_x;
        double lty = gamepad1.left_stick_y;
        boolean circle = gamepad1.circle;
        boolean cross = gamepad1.x;
        boolean square = gamepad1.y;
        boolean triangle = gamepad1.b; //triangle is used for test claw
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
        boolean x2= gamepad2.x;
        boolean y2 = gamepad2.y;
        boolean b2 = gamepad2.b;
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

        //PID Stuff
        firstStageController.setPID(p1,i1,d1);
        secondStageController.setPID(p2,i2,d2);
        int firstStagePos = firstStage.getCurrentPosition();
        int secondStagePos =firstStage.getCurrentPosition();
        double pid1 = firstStageController.calculate(firstStagePos,target1);
        double pid2 = secondStageController.calculate(secondStagePos,target2);
        double ff1 = Math.cos(Math.toRadians(target1 / ticks_in_degree))*f1;
        double ff2 = Math.cos(Math.toRadians(target2 / ticks_in_degree))*f2;
        double power1 = pid1 + ff1;
        double power2 = pid2 + ff2;

        firstStage.setPower(power1);
        secondStage.setPower(power2);
        telemetry.addData("pos1",firstStagePos);
        telemetry.addData("pos2", secondStagePos);

        telemetry.addData("target", target1);
        telemetry.addData("target2", target2);




        double powerfactor = 1;
        double rotate = -ltx;
        double x = -rtx;
        double y = -rty;
        double fleft = (x+y-rotate)*powerfactor;
        double fright = (-x+y+rotate)*powerfactor;
        double bleft = (-x+y-rotate)*powerfactor;
        double bright = (x+y+rotate)*powerfactor;


        lfront.setPower(fleft);
        rfront.setPower(fright);
        lback.setPower(bleft);
        rback.setPower(bright);
        if (b2){
            claw.setPosition(0.5);
        } else{
            claw.setPosition(0);
        }
        if (rt2>0.5){
           p1=0;
           i1=0;
           d1=0;
           target1=0;
           p2=0;
           i2=0;
           d2=0;
           target2=0;
        }










        telemetry.update();
    }

}

