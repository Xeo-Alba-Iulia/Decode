package org.firstinspires.ftc.teamcode.opmode

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.PI

@TeleOp(group = "Meet")
class BlueTeleOp : FullTeleOp() {
    override val startPose = Pose(60.0, 12.0, PI / 2)
    override val goalPose = Pose(4.0, 144.0 - 4.0)
    override val limelightPipeline = 1
}