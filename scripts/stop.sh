#!/bin/bash

PID_FILE=/home/cat/Codes/control/scripts/pidfile

# 检查 PID 文件是否存在
if [ -f $PID_FILE ]; then
  # 读取 PID
  PID=$(cat $PID_FILE)
  
  # 终止进程
  kill $PID
  
  # 删除 PID 文件
  rm $PID_FILE
else
  echo "PID file not found"
fi
