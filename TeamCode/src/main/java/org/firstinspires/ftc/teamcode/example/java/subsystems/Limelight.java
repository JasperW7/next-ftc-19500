package org.firstinspires.ftc.teamcode.example.java.subsystems;

import androidx.core.util.Supplier;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.BezierCurve;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroData;


import java.util.Arrays;
import java.util.concurrent.atomic.AtomicReference;

public class Limelight extends Subsystem {
    public static Limelight INSTANCE = new Limelight();

    private Limelight() {
    }

    public Limelight3A ll;
    public String name = "limelight";
    public LLResult result;
    private Pose samplePose = null;
    private double angle;

    private Command getSamplePose() {
        return new LambdaCommand()
                .setUpdate(() -> {
                    // Replace this with however you get the target pose.
                    // If the limelight output is null set samplePose to null,
                    // Otherwise set it to the pose
                    // You can have many lines of code here if needed.
                    result = ll.getLatestResult();
                    if (result== null) {
                        samplePose = null;
                    }else{
                        double[] python = result.getPythonOutput();
                        samplePose = new Pose(60-python[1],98+python[2],Math.toRadians(270));
                        angle = python[0];
                    }
                })
                .setIsDone(() -> samplePose != null);
    }

    private Command moveToSamplePath() {
        return new FollowPath(
                new PathBuilder()
                        .addPath(
                                new BezierLine(
                                        PedroData.INSTANCE.getFollower().getPose(),
                                        samplePose
                                )
                        )
                        .build()
        );
    }

    private Command createAndSchedule(Supplier<Command> factory) {
        AtomicReference<Command> command = new AtomicReference<>();
        return new LambdaCommand()
                .setStart(() -> {
                    command.set(factory.get());
                    command.get().invoke();
                })
                .setIsDone(() -> command.get().isDone());
    }

    public Command detectAndMove() {
        /*
         * Steps:
         * 1. Wait until we get a limelight output
         * 2. Convert that limelight output to a pose
         * 3. Create a FollowPath that moves from the current position to the detected pose
         */
        return new SequentialGroup(
                getSamplePose(),
                Rotation.INSTANCE.sub(angle),
                createAndSchedule(this::moveToSamplePath)
        );
    }

    @Override
    public void initialize(){
        ll = OpModeData.INSTANCE.getHardwareMap().get(Limelight3A.class, name);
    }
}
