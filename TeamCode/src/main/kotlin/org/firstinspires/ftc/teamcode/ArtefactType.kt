package org.firstinspires.ftc.teamcode

enum class ArtefactType {
    GREEN, PURPLE
}

fun Int.toArtefactList(): List<ArtefactType> = when (this) {
    21 -> listOf(GREEN, PURPLE, PURPLE)
    22 -> listOf(PURPLE, GREEN, PURPLE)
    23 -> listOf(PURPLE, PURPLE, GREEN)
    else -> error("Invalid fiducialId: $this")
}