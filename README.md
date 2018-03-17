# Public course from pony.ai
In this course you'll learn basic knowledge about autonomous car and complete essential algorithms
in autonomous car simulation system. In next weeks, you'll get some real data recorded during
our road test. Your task is writing code to recognise around obstacles, plan a best path to
avoid them and make autonomous car run along the road for its destination.

You can access the code and document for homework here.

Let's begin now!!!

# System setup

A shared display library with some utility classes has been introduced into the codebase. We will provide several visualization tools based on this tool. To use this tool, you need to run following commands to install several system dependencies. 

```
sudo apt install qtdeclarative5-dev clang-3.8 nasm
```

**QT** is a cross-platform application framework. Our visualization tools will be developed based on it. 

**Clang** is the compiler we use to build the libraries. We require you to install `clang-3.8` to avoid any potential issues caused by compiler version. 

**nasm** is a required library for an introduced third-party library. 
