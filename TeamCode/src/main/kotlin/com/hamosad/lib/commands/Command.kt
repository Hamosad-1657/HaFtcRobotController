package com.hamosad.lib.commands

abstract class Command {
    abstract val requirements: List<Subsystem>

    abstract fun initialize()
    abstract fun execute()
    abstract fun onEnd(wasInterrupted: Boolean)
    abstract fun isFinished(): Boolean


}