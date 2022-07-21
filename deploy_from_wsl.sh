#!/usr/bin/sh

if [ "$#" -ne 2 ]; then
    echo "Usage: $0 COMn driveletter"
    exit 1
fi

cd build
if ! make; then
    echo "make failed! Skipping copy"
    return $?
fi

powershell.exe -Command "\$port= new-Object System.IO.Ports.SerialPort $1,1200,None,8,one;\$port.Open();"
sleep 1
cmd.exe /c "copy `wslpath -w ./co2mon.uf2` $2:\\ "
