#!/bin/sh
trap : TERM INT
tail -f /dev/null & wait
