package org.firstinspires.ftc.teamcode.opmode

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.sorter.Sorter
import org.firstinspires.ftc.teamcode.sorter.SorterException

@TeleOp(group = "Tests")
class SorterOpMode : OpMode() {
    lateinit var sorter: Sorter

    override fun init() {
        sorter = Sorter(hardwareMap)
    }

    override fun loop() {
        try {
            when {
                gamepad1.crossWasPressed() -> sorter.intakeBall()
                gamepad1.circleWasPressed() -> sorter.getBall()
                gamepad1.right_bumper -> sorter.resetPosition()
            }
        } catch (exception: SorterException) {
            Log.e("SorterOpMode", "Error in sorter operation", exception)
        }
    }
}