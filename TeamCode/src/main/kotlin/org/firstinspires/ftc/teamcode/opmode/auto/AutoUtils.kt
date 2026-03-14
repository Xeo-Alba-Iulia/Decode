package org.firstinspires.ftc.teamcode.opmode.auto

import com.pedropathing.geometry.Pose

fun Pose.mirrorAlliance(isMirrored: Boolean): Pose = if (isMirrored) pose.mirror() else pose
