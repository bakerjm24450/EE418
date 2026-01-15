# _EE 418 Homework 8_

The goal of this assignment is to gain some experience with FreeRTOS task notifications.

Write a program that creates a FreeRTOS task and a FreeRTOS timer with a 500 ms period. The timer callback should send a task notification to the task. In the task function (inside the infinite loop), the task should wait for the notification, toggle an LED, and repeat.