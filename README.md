StateOS
=======

Free, extremely simple and amazingly tiny real-time operating system (RTOS) designed for deeply embedded applications.
It was inspired by the concept of a state machine.
TEMPLATE. NOT TESTED!

Features
--------

- kernel works in preemptive or cooperative mode
- kernel can operate in tick-less mode (32-bit timer required)
- signals (auto clearing, protected)
- events
- flags (one, all, accept, ignore)
- barriers
- semaphores (binary, limited, counting)
- mutexes (normal, recursive, priority protect, priority inheritance)
- condition variables
- message queues
- mailbox queues
- timers (one-shot, periodic)
- c++ wrapper
- all documentation is contained within the source files

Target
-------

STM32F3Discovery board.

License
-------

This project is licensed under the terms of [GNU GENERAL PUBLIC LICENSE Version 3](http://www.gnu.org/philosophy/why-not-lgpl.html).
