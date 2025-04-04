package org.firstinspires.ftc.teamcode.example.java.subsystems;

import static com.rowanmcalpin.nextftc.ftc.OpModeData.telemetry;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorGroup;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;


public class Slide extends Subsystem {
    // BOILERPLATE
    public static final Slide INSTANCE = new Slide();
    private Slide() { }

    // USER CODE
    public MotorEx motor,motor2;
    public MotorGroup slides;
    public PIDFController controller = new PIDFController(0.017, 0.0, 0.00018);
    public Command resetZero() {
        return new InstantCommand(() -> { motor.resetEncoder(); });
    }

    public String name = "S1Motor",name2 = "S2Motor";

    public Command toUp() {
        return new RunToPosition(slides, // MOTOR TO MOVE
                1300, // TARGET POSITION, IN TICKS
                controller, // CONTROLLER TO IMPLEMENT
                this); // IMPLEMENTED SUBSYSTEM
    }


    public Command toRest(){
        return new RunToPosition(slides,
                100,
                controller,
                this);
    }

    public Command down(){
        return new RunToPosition(slides,
                500,
                controller,
                this
                );
    }

    public double getPos(){
        return motor.getCurrentPosition();
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
