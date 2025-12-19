#!/bin/bash
chmod o=wr /dev/ttyS0
chmod o=wr /dev/ttyS7

sleep 2

ROOT_DIR=/home/cat/Codes/control
PID_FILE=$ROOT_DIR/scripts/pidfile

cd $ROOT_DIR
export PYTHONPATH=$(sudo -u cat python3 -c "import sys; print(':'.join(sys.path))")
python3 api.py &

echo $! > $PID_FILE
