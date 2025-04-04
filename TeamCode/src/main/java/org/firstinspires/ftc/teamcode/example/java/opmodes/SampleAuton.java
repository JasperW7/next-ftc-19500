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

@Autonomous(name = "Sample Auton", group = "Autonomous")
public class SampleAuton extends PedroOpMode {
    public SampleAuton() {
        super(Claw.INSTANCE, Wrist.INSTANCE, Rotation.INSTANCE, Pivot.INSTANCE, Slide.INSTANCE, Limelight.INSTANCE);
    }

    public double x,y,angle;

    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */
    private final Pose scorePose = new Pose(19, 124, Math.toRadians(315));

    /** Lowest (First) Sample from the Spike Mark */
    private final Pose pickup1Pose = new Pose(26, 121, Math.toRadians(0));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(26, 131, Math.toRadians(0));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(28, 133, Math.toRadians(30));

    /** Park Pose for our robot, after we do all of the scoring. */
    public final Pose parkPose = new Pose(60, 98, Math.toRadians(270));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(60, 115, Math.toRadians(90));

    public Path park;
    public Path grabFromSub;
    private PathChain scorePreload,grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    public void buildPaths(){
        scorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose),   new Point(scorePose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose),   new Point(pickup1Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup2Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup2Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup3Pose)))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup3Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        park = new Path(new BezierCurve(new Point(scorePose), new Point(parkControlPose), new Point(parkPose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading());

        grabFromSub = new Path(new BezierCurve(
                new Point(parkPose),
                new Point(parkPose.getX() + 5, parkPose.getY())
        ));
        grabFromSub.setLinearHeadingInterpolation(parkPose.getHeading(), Math.toRadians(270));

    }

    public Command firstRoutine() {
        return new SequentialGroup(
                // PRELOAD
                new ParallelGroup(
                        Wrist.INSTANCE.par(),
                        Rotation.INSTANCE.normal(),
                        Claw.INSTANCE.close(),
                        Pivot.INSTANCE.toHigh(), //arm up
                        new FollowPath(scorePreload), //move while arm is going up
                        new SequentialGroup(
                                new WaitUntil(() -> Pivot.INSTANCE.getPos() > 700), // also bring slide up after arm goes up
                                Slide.INSTANCE.toUp()
                        )
                ),
                Wrist.INSTANCE.outtake().thenWait(.5),
                Claw.INSTANCE.open().thenWait(.5),
                // GRAB 1
                new ParallelGroup(
                        Wrist.INSTANCE.par(), //wrist par to go back
                        Slide.INSTANCE.down(), //partial slide down
                        new FollowPath(grabPickup1), //move on path alongside
                        new SequentialGroup(
                                new WaitUntil(() -> Slide.INSTANCE.getPos() > 120),
                                Pivot.INSTANCE.toRest() //pivot down
                        )
                ).thenWait(1),
                Pivot.INSTANCE.toDown().thenWait(.2), //bring pivot down to grab
                Claw.INSTANCE.close().thenWait(.3), //close claw

                // SCORE 1
                new ParallelGroup(
                        Pivot.INSTANCE.toHigh(), //bring pivot back up to score
                        new FollowPath(scorePickup1), //follow path alongside
                        new SequentialGroup(
                                new WaitUntil(() -> Pivot.INSTANCE.getPos()>700),
                                Slide.INSTANCE.toUp() //also bring slide up when pivot is finished
                        )
                ),
                Wrist.INSTANCE.outtake().thenWait(0.5), // wrist pos to score
                Claw.INSTANCE.open().thenWait(0.5), // open claw

                // GRAB 2
                new ParallelGroup(
                        Wrist.INSTANCE.par(), //wrist par to go back
                        Slide.INSTANCE.down(), //partial slide down
                        new FollowPath(grabPickup2), //move on path alongside
                        new SequentialGroup(
                                new WaitUntil(() -> Slide.INSTANCE.getPos() > 120),
                                Pivot.INSTANCE.toRest() //pivot down
                        )
                ).thenWait(1),
                Pivot.INSTANCE.toDown().thenWait(.2), //bring pivot down to grab
                Claw.INSTANCE.close().thenWait(.3), //close claw

                // SCORE 2
                new ParallelGroup(
                        Pivot.INSTANCE.toHigh(), //bring pivot back up to score
                        new FollowPath(scorePickup2), //follow path alongside
                        new SequentialGroup(
                                new WaitUntil(() -> Pivot.INSTANCE.getPos()>700),
                                Slide.INSTANCE.toUp() //also bring slide up when pivot is finished
                        )
                ),
                Wrist.INSTANCE.outtake().thenWait(0.5), // wrist pos to score
                Claw.INSTANCE.open().thenWait(0.5), // open claw

                // GRAB 3
                new ParallelGroup(
                        Wrist.INSTANCE.par(), //wrist par to go back
                        Slide.INSTANCE.down(), //partial slide down
                        new FollowPath(grabPickup3), //move on path alongside
                        new SequentialGroup(
                                new WaitUntil(() -> Slide.INSTANCE.getPos() > 120),
                                Pivot.INSTANCE.toRest() //pivot down
                        )
                ).thenWait(1),
                Pivot.INSTANCE.toDown().thenWait(.2), //bring pivot down to grab
                Claw.INSTANCE.close().thenWait(.3), //close claw

                // SCORE 3
                new ParallelGroup(
                        Pivot.INSTANCE.toHigh(), //bring pivot back up to score
                        new FollowPath(scorePickup3), //follow path alongside
                        new SequentialGroup(
                                new WaitUntil(() -> Pivot.INSTANCE.getPos()>700),
                                Slide.INSTANCE.toUp() //also bring slide up when pivot is finished
                        )
                ),
                Wrist.INSTANCE.outtake().thenWait(.5),
                Claw.INSTANCE.sub().thenWait(.5),

                new FollowPath(park),
                Limelight.INSTANCE.detectAndMove()
                


        );
    }



    @Override
    public void onInit(){
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);
        follower.setStartingPose(startPose);

        buildPaths();
    }
    @Override
    public void onStartButtonPressed() {
        firstRoutine().invoke();
    }
}
