# crocodile-uhf-app



## Install GEOSURV SDK

Download SDK from TC, here is link of 3.1

https://teamcity-hbg.leica-geosystems.com/repository/download/BspYocto_GeoSurvBuildCrocodile/4584418:id/geosurv-aarch64-x86_64-leica-image-dev-crocodile-toolchain-3.1.sh

install SDK

```
 ./geosurv-aarch64-x86_64-leica-image-dev-crocodile-toolchain-3.1.sh
GeoSurv SDK installer version 3.1
=================================
Enter target directory for SDK (default: /opt/geosurv/3.1):
You are about to install the SDK to "/opt/geosurv/3.1". Proceed [Y/n]? y
[sudo] password for liqiw:
Extracting SDK................done
Setting it up...done

```

## build crocodile-uhf-app

```
git clone https://github.com/Qingwu-Li/crocodile-uhf-app.git
cd crocodile-uhf-app
source /opt/geosurv/3.1/environment-setup-aarch64-leicageo-linux 
make clean
make all
```



## copy spirit1_test to EVK

```
scp ./bin/spirit1_test root@10.60.34.93:/tmp/
scp ./bin/spirit1_test root@10.60.34.54:/tmp/
```



## run test on EVK

Transmitter command:

```
./spirit1_test -t 50 -i 10000 -e 76800 -p -10
```

-t 50 ,		 Transmitter , package size 50.

-i 10000,	Test run 10000 integrations 

-e 76800,   Air datarate 76800

-p -10,       Max power lever -10dm

Receiver Command:

```
./spirit1_test -r -e 76800 -p -10
```

-r                Receiver 

-e 76800,   Air datarate 76800

-p -10,       Max power lever -10dm



## Usage

```
usage: ./spirit1_test [options] ...
options:
  -c, --channel           Channel number (int [=0])
  -e, --datarate          Air dataratE [100, 500000] (int [=38400])
  -b, --bandwidth         Bandwidth [1100, 800100] (int [=100000])
  -a, --addr              my Address (int [=52])
  -d, --dest              Destination address (int [=68])
  -i, --iterate           iterates of the test (int [=1])
  -w, --waittime          the wait ms between two packages(transmitter) (int [=1])
  -p, --powerlevel        Output power leve[-34,11] (int [=0])
  -t, --transmitlength    Transmit buffer length[2,60] (int [=2])
  -r, --receiver          as a receiver
  -v, --verbose           enable verbose message
  -?, --help              print this message

```



# Hardware setup for IMX8MM EVK

| IMX8MM Name | EVK J1003 Pin Number | IDS01A4 | IDS01A4 CN |
| ----------- | -------------------- | ------- | ---------- |
| ECSPI2_MOSI | 19                   | PA7     | CN5.4      |
| ECSPI2_MISO | 21                   | PA6     | CN5.5      |
| ECSPI2_SCL  | 23                   | PB3     | CN9.4      |
| ECSPI2_SS0  | 24                   | PB6     | CN5.3      |
| SAI5_RXC    | 40                   | PA10    | CN9.3      |
| SAI5_RXD0   | 38                   | PC7     | CN5.2      |


