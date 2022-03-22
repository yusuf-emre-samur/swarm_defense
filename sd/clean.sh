#!/bin/bash
# kill server and client
if pgrep -x "gzclient" > /dev/null
then
    kill $(pgrep gzclient)
fi
if pgrep -x "gzserver" > /dev/null
then
    kill $(pgrep gzserver)
fi
