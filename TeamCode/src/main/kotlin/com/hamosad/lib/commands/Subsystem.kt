package com.hamosad.lib.commands

abstract class Subsystem {
    abstract var defaultCommand: Command?
    abstract fun periodic()
}