package org.firstinspires.ftc.teamcode.example.java;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.Controllable;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;


public class Slide extends Subsystem {
    // BOILERPLATE
    public static final Slide INSTANCE = new Slide();
    private Slide() { }

    // USER CODE
    public DcMotorEx motor,motor2;

    public PIDFController controller = new PIDFController(0.017, 0.0, 0.00018);
    public Command resetZero() {
        return new InstantCommand(() -> { motor.resetEncoder(); });
    }

    public String name = "S1Motor",name2 = "S2Motor";

    public Command toUp() {
        return new RunToPosition((Controllable) motor, // MOTOR TO MOVE
                1400, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
                new RunToPosition((Controllable) motor2,
                1400,
                controller,
                this);
    }

    public Command toRest(){
        return new RunToPosition((Controllable) motor,
                100,
                controller,
                this);
                new RunToPosition((Controllable) motor2,
                100,
                controller,
                this);
    }



    @Override
    public void initialize() {
        motor = new DcMotorEx(name);
        motor2 = new DcMotorEx(name2);
    }
}
