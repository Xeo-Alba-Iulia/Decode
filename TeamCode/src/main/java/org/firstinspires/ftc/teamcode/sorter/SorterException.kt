package org.firstinspires.ftc.teamcode.sorter

sealed class SorterException(message: String) : Exception(message)

class EmptySorter : SorterException("Empty Sorter")
class FullSorter : SorterException("Full Sorter")