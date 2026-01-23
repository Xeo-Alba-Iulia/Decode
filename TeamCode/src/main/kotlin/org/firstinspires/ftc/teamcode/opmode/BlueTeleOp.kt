package org.firstinspires.ftc.teamcode.opmode

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import kotlin.math.PI

@TeleOp(group = "Meet")
class BlueTeleOp : FullTeleOp() {
    override val startPose = Pose(24.0 * 2 + 12.0, 12.0, PI / 2)
    override val goalPose = Pose(10.0, 144.0 - 10.0)
}