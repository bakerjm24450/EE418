# _EE 418 Homework 9_

The goal of this assignment is to gain some experience with FreeRTOS event groups.

Write a program with two tasks, each of which controls a separate LED. Each task should delay for a random time (between 1 and 2 seconds), turn their LED on, delay again for a random time, and then set a separate bit in the event group. Both tasks should wait until the other task has posted its event, and then each should turn off their LED.

When rebooted, the expected output is that the two LEDs will turn on at different times, and then both turn off at the same time as the tasks are synchronized.