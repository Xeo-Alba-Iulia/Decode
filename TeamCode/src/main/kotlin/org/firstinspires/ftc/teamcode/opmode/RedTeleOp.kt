package org.firstinspires.ftc.teamcode.opmode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Alliance

@TeleOp(group = "Meet")
class RedTeleOp : FullTeleOp(Alliance.RED, limelightPipeline = 2)