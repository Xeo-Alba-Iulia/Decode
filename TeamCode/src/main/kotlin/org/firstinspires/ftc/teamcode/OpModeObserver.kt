package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode

interface OpModeObserver {
    suspend fun onStart(opMode: OpMode) {}
    suspend fun onStop(opMode: OpMode) {}
}