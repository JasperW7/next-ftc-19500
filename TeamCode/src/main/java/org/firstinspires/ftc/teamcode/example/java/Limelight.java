package org.firstinspires.ftc.teamcode.example.java;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

import java.util.Arrays;

public class Limelight extends Subsystem {
    // BOILERPLATE
    public static final Limelight INSTANCE = new Limelight();
    private Limelight() { }

    // USER CODE
    public Limelight3A ll;
    public LLResult result;
    public String name = "limelight";


    public double[] getResult(){
        ll.start();
        result = ll.getLatestResult();
        if (result!=null){
            double[] pythonOutput = result.getPythonOutput();
            ll.pause();
            return Arrays.copyOfRange(pythonOutput, 4, 7);
        }else{
            return null;
        }
    }




    @Override
    public void initialize() {
        ll = OpModeData.INSTANCE.getHardwareMap().get(Limelight3A.class, name);
        ll.pipelineSwitch(1);
        ll.pause();
    }
}
