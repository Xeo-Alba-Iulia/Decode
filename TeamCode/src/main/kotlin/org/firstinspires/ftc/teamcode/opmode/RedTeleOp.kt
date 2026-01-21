package org.firstinspires.ftc.teamcode.opmode

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.PI

@TeleOp(group = "Meet")
class RedTeleOp : FullTeleOp() {
    override val startPose = Pose(72.0 + 12.0, 12.0, PI / 2)
    override val goalPose = Pose(144.0, 144.0)
}