#!/bin/sh

echo "Cleaning map"
sh ./cleanMap.sh

# Extract and prepare map
echo "Preparing map"
./external/osrm-backend/build/osrm-extract -p external/osrm-backend/profiles/car.lua maps/us-ne/us-northeast-latest.osm.pbf
./external/osrm-backend/build/osrm-partition maps/us-ne/us-northeast-latest
./external/osrm-backend/build/osrm-customize maps/us-ne/us-northeast-latest


