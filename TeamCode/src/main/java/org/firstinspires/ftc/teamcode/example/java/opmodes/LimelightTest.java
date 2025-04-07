package org.firstinspires.ftc.teamcode.example.java.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelDeadlineGroup;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;
import com.rowanmcalpin.nextftc.pedro.FollowPath;
import com.rowanmcalpin.nextftc.pedro.PedroOpMode;

import org.firstinspires.ftc.teamcode.example.java.FConstants;
import org.firstinspires.ftc.teamcode.example.java.LConstants;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Claw;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Rotation;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Slide;
import org.firstinspires.ftc.teamcode.example.java.subsystems.Wrist;

@Autonomous(name = "Limelight", group = "Autonomous")
public class LimelightTest extends PedroOpMode {
    public LimelightTest() {
        super(Claw.INSTANCE, Wrist.INSTANCE, Rotation.INSTANCE, Pivot.INSTANCE, Slide.INSTANCE, Limelight.INSTANCE);
    }

    /** Park Pose for our robot, after we do all of the scoring. */
    public final Pose parkPose = new Pose(60, 98, Math.toRadians(270));


    public Path grabFromSub;
    private PathChain scorePreload,grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3,sub1,scoresub1;

    public void buildPaths() {

    }
    public Command firstRoutine() {
        return new SequentialGroup(
                new ParallelGroup(
                        Pivot.INSTANCE.toRest(),
                        Slide.INSTANCE.toRest(),
                        Wrist.INSTANCE.par(),
                        Rotation.INSTANCE.normal(),
                        Claw.INSTANCE.open()

                ),
                Limelight.INSTANCE.detectAndMove(),
                Rotation.INSTANCE.sub(Limelight.INSTANCE.angle),
                Pivot.INSTANCE.toDown(),
                Claw.INSTANCE.close()



        );

    }



    @Override
    public void onInit(){
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(parkPose);
        Slide.INSTANCE.resetZero();
        Pivot.INSTANCE.resetZero();
        buildPaths();

    }
    @Override
    public void onStartButtonPressed() {
        firstRoutine().invoke();
    }
}
