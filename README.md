# osrm-data-generator

Script for generating trayectories from a csv file using OSRM for route tracing.

#### Description

This code uses the OSRM library to generate the trayectories of routes between 2 points. The trayectories are represented by the list of OSRM node ids along the route and end in 0. Ex.:

4782506033 6814933411 805764932 10693951249 0

It also generates a file with the duration of the trip rounded to an unsigned integer in seconds in each node, with the first node always strating in 0 and the end of the trip ending in UINT32_MAX (coinciding with the 0 in the trayectories output file). Ex.:

0 1 3 5 4294967295

Note: lines 252 and 255 of /src/generate-data.cpp use a map to transform the id nodes into smaller numbers. If you want the original nodes modify this lines. 

Along with those two files, it also prints a csv with the id of the object in the trip, the coordinates of origin and destination and the times of departure and arrival (expresed in seconds since the date defined in the constant EDATE in generate_data.cpp).

A file with the lines of the csv that couldn't be processed it's also generated with the line number and the reason.

#### Prerequisites

- git
- cmake
- gcc/g++
- liblua5.2-dev
- libboost-all-dev
- libprotobuf-dev
- libtbb-dev
- libstxxl-dev
- libbz2-dev

```
sudo apt install -y git cmake build-essential
sudo apt install -y liblua5.2-dev libboost-all-dev libprotobuf-dev libtbb-dev libstxxl-dev libbz2-dev
```

#### Installation

- git clone 
- cd osrm-data-generator
- ./compile.sh (if needed, use chmod +x path/compile.sh)
    - this will compile the OSRM backend and the generator
- ./build.sh (if needed, use chmod +x path/compile.sh)
    - this will extract the us northeast map in ./maps/us-ne and create the structures needed for tracing the routes

#### Docker

Situate in the root directory of the repository and run:

```
docker build -t osrm-generator .
```

After that run the generator with:

```
docker run -v </path/to/csv/with/data>:/csv -v </path/to/output/dir>:/out osrm-generator /csv/<csv_name.csv> /data/maps/us-ne/us-northeast-latest /out
```
