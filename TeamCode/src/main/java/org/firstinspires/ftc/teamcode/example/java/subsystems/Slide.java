package org.firstinspires.ftc.teamcode.example.java.subsystems;


import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Slide extends Subsystem {
    // BOILERPLATE
    public static final Slide INSTANCE = new Slide();
    private Slide() { }

    // USER CODE
    public Telemetry telemetry;
    public MotorEx motor,motor2;
    public double target;
    public MotorGroup slides;
    public PIDFController controller = new PIDFController(0.017, 0.0, 0.00018, new StaticFeedforward(0.1),100);
    public Command resetZero() {
        return new InstantCommand(() -> { motor.resetEncoder(); motor2.resetEncoder();});
    }

    public String name = "S1Motor",name2 = "S2Motor";

    public Command toUp() {
        target = 1400;
        return new RunToPosition(slides, // MOTOR TO MOVE
                target, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }


    public Command toRest(){
        target =100;
        return new RunToPosition(slides,
                target,
                controller,
                this);
    }


    public Command toThird(){
        target = 320;
        return new RunToPosition(
                slides,
                target,
                controller,
                this
        );
    }

    public Command down(){
        target = 400;
        return new RunToPosition(slides,
                target,
                controller,
                this
                );
    }

    public double getPos(){
        return slides.getCurrentPosition();
    }

    @Override
    public Command getDefaultCommand() {
        return new HoldPosition(slides, controller, this);
    }



    @Override
    public void initialize() {
        motor = new MotorEx(name);
        motor2 = new MotorEx(name2);
        slides = new MotorGroup(motor,motor2);
    }
}
