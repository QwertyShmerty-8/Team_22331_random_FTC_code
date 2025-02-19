package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp
public class TempEncoder extends OpMode {
    public void init(){

    }
    public void loop(){
        AnalogInput encoder = hardwareMap.get(AnalogInput.class,"encoder");
        AnalogInput encoder2 = hardwareMap.get(AnalogInput.class,"encoder2");
        double position = encoder.getVoltage() /3.2*360;
        telemetry.addData("angle:", position);
        double position2 = encoder2.getVoltage()/3.2 *360;
        telemetry.addData("angle2:", position2);
    }

}
