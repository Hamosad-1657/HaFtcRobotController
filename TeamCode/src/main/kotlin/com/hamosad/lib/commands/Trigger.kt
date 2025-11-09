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
class Trigger(private val condition: () -> Boolean) {
    init {
        CommandScheduler.registerTrigger(this)
    }
    private val bindings: MutableMap<BindingType, MutableList<Command>> = mutableMapOf(
        Pair(BindingType.ON_TRUE, mutableListOf()),
        Pair(BindingType.WHILE_TRUE, mutableListOf()),
        Pair(BindingType.TOGGLE_ON_TRUE, mutableListOf()),
        Pair(BindingType.ON_FALSE, mutableListOf()),
        Pair(BindingType.WHILE_FALSE, mutableListOf()),
        Pair(BindingType.TOGGLE_ON_FALSE, mutableListOf()),
    )

    private var last = false
    fun evaluate() {
        val current = condition()
        val rising = !last && current
        val falling = last && !current

        for (binding in bindings) {
            val commands = binding.value
            when (binding.key) {
                BindingType.ON_TRUE ->
                    if (rising) {
                        for (command in commands) {
                            CommandScheduler.scheduleCommand(command)
                        }
                    }
                BindingType.WHILE_TRUE ->
                    if (rising) {
                        for (command in commands) {
                            CommandScheduler.scheduleCommand(command)
                        }
                    } else if (falling) {
                        for (command in commands) {
                            CommandScheduler.endCommand(command)
                        }
                    }
                BindingType.TOGGLE_ON_TRUE ->
                    if (rising) {
                        for (command in commands) {
                            CommandScheduler.toggleCommand(command)
                        }
                    }

                BindingType.ON_FALSE ->
                    if (falling) {
                        for (command in commands) {
                            CommandScheduler.scheduleCommand(command)
                        }
                    }
                BindingType.WHILE_FALSE ->
                    if (falling) {
                        for (command in commands) {
                            CommandScheduler.scheduleCommand(command)
                        }
                    } else if (rising) {
                        for (command in commands) {
                            CommandScheduler.endCommand(command)
                        }
                    }
                BindingType.TOGGLE_ON_FALSE ->
                    if (falling) {
                        for (command in commands) {
                            CommandScheduler.toggleCommand(command)
                        }
                    }
            }
        }

        last = current
    }

    fun onTrue(command: Command) {
        bindings[BindingType.ON_TRUE]?.add(command)
    }

    fun whileTrue(command: Command) {
        bindings[BindingType.WHILE_TRUE]?.add(command)
    }

    fun toggleOnTrue(command: Command) {
        bindings[BindingType.TOGGLE_ON_TRUE]?.add(command)
    }

    fun onFalse(command: Command) {
        bindings[BindingType.ON_FALSE]?.add(command)
    }

    fun whileFalse(command: Command) {
        bindings[BindingType.WHILE_FALSE]?.add(command)
    }

    fun toggleOnFalse(command: Command) {
        bindings[BindingType.TOGGLE_ON_FALSE]?.add(command)
    }
}