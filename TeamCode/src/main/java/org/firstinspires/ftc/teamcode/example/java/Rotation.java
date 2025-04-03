package org.firstinspires.ftc.teamcode.example.java;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class Rotation extends Subsystem {
    // BOILERPLATE
    public static final Rotation INSTANCE = new Rotation();
    private Rotation() { }

    // USER CODE
    public Servo servo;

    public String name = "rotation";

    public Command normal() {
        return new ServoToPosition(servo, // SERVO TO MOVE
                .46, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command third() {
        return new ServoToPosition(servo, // SERVO TO MOVE
                0.75, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command sub(double angle){
        return new ServoToPosition(servo,
                angle,
                this);
    }


    @Override
    public void initialize() {
        servo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
    }
}
