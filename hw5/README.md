# _EE 418 Homework 5_

The goal of this assignment is to gain some experience with FreeRTOS queues.

Write a program with two tasks that use a queue to communicate in a producer-consumer pattern. The producer task will generate a sequence of random numbers, passing them to the consumer task. The consumer will use the received value as a delay time for blinking two LEDs.