# _EE 418 Homework 6_

The goal of this assignment is to gain some experience with FreeRTOS timers.

Write a program that will toggle 2 GPIO pins with a frequency of 500 Hz. One of the pins will be controlled by a FreeRTOS timer and the second by a task. You will be able to observe the difference in jitter (or steadiness) for the two approaches as the CPU becomes busy with other work.