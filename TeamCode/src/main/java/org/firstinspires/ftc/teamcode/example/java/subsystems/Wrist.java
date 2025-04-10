package org.firstinspires.ftc.teamcode.example.java.subsystems;

import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition;

public class Wrist extends Subsystem {
    // BOILERPLATE
    public static final Wrist INSTANCE = new Wrist();
    private Wrist() { }

    // USER CODE
    public Servo servo;

    public String name = "wrist";

    public Command perp() {
        return new ServoToPosition(servo, // SERVO TO MOVE
                0.62, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command par() {
        return new ServoToPosition(servo, // SERVO TO MOVE
                0.1, // POSITION TO MOVE TO
                this); // IMPLEMENTED SUBSYSTEM
    }
    public Command outtake() {
        return new ServoToPosition(servo,
                .82,
                this);
    }

    @Override
    public void initialize() {
        servo = OpModeData.INSTANCE.getHardwareMap().get(Servo.class, name);
    }
}
