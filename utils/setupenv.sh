#!/bin/bash

if [ "$#" -ne 1 ]; then
	echo "Usage : $0 <PATH TO PDK FOLDER>"
	exit 1
fi
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"
patch $1/packages/ti/drv/uart/src/v1/UART_v1.c $DIR/UART_v1.patch
if [ $? -ne 0 ]; then
	echo "PATCH failed, log at UART_v1.c.rej"
    exit 1
fi

patch $1/packages/ti/drv/uart/src/UART_drv.c $DIR/UART_drv.patch
if [ $? -ne 0 ]; then
	echo "PATCH failed, log at UART_drv.c.rej"
    exit 1
fi

echo "Patch applied, gonna rebuild the SDK"
pushd $1/packages
make uart
popd
echo "Build succeed"
exit 0
