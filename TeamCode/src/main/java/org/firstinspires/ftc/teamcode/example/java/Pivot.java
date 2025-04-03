package org.firstinspires.ftc.teamcode.example.java;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.Controllable;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;


public class Pivot extends Subsystem {
    // BOILERPLATE
    public static final Pivot INSTANCE = new Pivot();
    private Pivot() { }

    // USER CODE
    public MotorEx motor;

    public PIDFController controller = new PIDFController(0.03, 0.0, 0.0,new StaticFeedforward(0));
    public PIDFController controllerLow = new PIDFController(0.01,0,0,new StaticFeedforward(0.005));
    public Command resetZero() {
        return new InstantCommand(() -> { motor.resetEncoder(); });
    }

    public String name = "AMotor";


    public Command toRest(){
        return new RunToPosition(motor,
                100,
                controller,
                this);
    }

    public Command toDown() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                25, // TARGET POSITION, IN TICKS
                controllerLow, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public Command toHigh() {
        return new RunToPosition(motor, // MOTOR TO MOVE
                890.0, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }

    public double getPos(){
        return motor.getCurrentPosition();
    }
    
    @Override
    public void initialize() {
        motor = new MotorEx(name);
    }
}
