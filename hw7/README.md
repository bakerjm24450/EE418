# _EE 418 Homework 7_

The goal of this assignment is to gain some experience with FreeRTOS mutexes.

You are given a program that creates 4 tasks, each that performs some computation and writes output to stdout (printf). Since stdout is, in effect, a shared resource among the tasks, you will observe that the tasks interfere with each other -- the output messages are all inter-mixed.

Modify the program to ensure mutually exclusive access to the stdout console using a mutex.