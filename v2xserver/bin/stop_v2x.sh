#!/bin/bash

program_name="v2xServ"


pid=$(pgrep $program_name)

if [ -z "$pid" ]; then
    echo "程序 $program_name 未在运行中"
else
    echo "停止程序 $program_name，PID: $pid"
    kill -15 $pid
fi

