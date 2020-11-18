# crocodile-uhf-app



## Install geosurv SDK

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
Extracting SDK............................................................................................................................................................done
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

test with datarate 9600 bandwidth    2000

with option -r indicate as a receiver

with option -t indicate as a transmitter

```
./spirit1_test -r -e 9600 -b 200000
./spirit1_test -t 1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMN -i 100 -e 9600 -b 200000
```



## Usage

```
spirit1_test -h
undefined short option: -h
usage: ./spirit1_test [options] ...
options:
  -c, --channel      Channel number (int [=0])
  -e, --datarate     Air dataratE [100, 500000] (int [=38400])
  -b, --bandwidth    Bandwidth [1100, 800100] (int [=100000])
  -a, --addr         my Address (int [=52])
  -d, --dest         Destination address (int [=68])
  -i, --iterate      iterates of the test (int [=1])
  -r, --receiver     As a receiver
  -v, --verbose      enable verbose message
  -t, --data         string to send via uhf (string [=])
  -?, --help         print this message
```

