package com.hamosad.lib.commands

enum class BindingType {
    ON_TRUE,
    WHILE_TRUE,
    TOGGLE_ON_TRUE,

    ON_FALSE,
    WHILE_FALSE,
    TOGGLE_ON_FALSE,
}

/** Stores a condition that the trigger is true when it is true, and otherwise false. Allows you to set bindings from the trigger to a command. */
class Trigger(val condition: () -> Boolean) {
    private val bindings: MutableMap<BindingType, Command> = mutableMapOf()


    private var last = false
    fun evaluate() {
        val current = condition()
        val rising = !last && current
        val falling = last && !current

        for (binding in bindings) {
            val command = binding.value
            when (binding.key) {
                BindingType.ON_TRUE ->
                    if (rising) {
                        CommandScheduler.scheduleCommand(command)
                    }
                BindingType.WHILE_TRUE ->
                    if (rising) {
                        CommandScheduler.scheduleCommand(command)
                    } else if (falling) {
                        CommandScheduler.endCommand(command)
                    }
                BindingType.TOGGLE_ON_TRUE ->
                    if (rising) {
                        CommandScheduler.toggleCommand(command)
                    }

                BindingType.ON_FALSE ->
                    if (falling) {
                        CommandScheduler.scheduleCommand(command)
                    }
                BindingType.WHILE_FALSE ->
                    if (falling) {
                        CommandScheduler.scheduleCommand(command)
                    } else if (rising) {
                        CommandScheduler.endCommand(command)
                    }
                BindingType.TOGGLE_ON_FALSE ->
                    if (falling) {
                        CommandScheduler.toggleCommand(command)
                    }
            }
        }

        last = current
    }
}