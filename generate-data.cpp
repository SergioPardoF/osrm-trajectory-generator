#include "osrm/match_parameters.hpp"
#include "osrm/nearest_parameters.hpp"
#include "osrm/route_parameters.hpp"
#include "osrm/table_parameters.hpp"
#include "osrm/trip_parameters.hpp"

#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"

#include "osrm/osrm.hpp"
#include "osrm/status.hpp"

#include <exception>
#include <iostream>
#include <string>
#include <utility>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>
#include <sys/stat.h>
#include <chrono>

#include <cstdlib>

// Function to split a string based on delimiter
std::vector<std::string> splitString(const std::string& line, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

// Add seconds to String Date in format "Y-m-d H:M:S"
std::string addSecondsToDate(const std::string& dateString, int secondsToAdd) {
    std::tm timeStruct = {};
    std::istringstream ss(dateString);
    ss >> std::get_time(&timeStruct, "%Y-%m-%d %H:%M:%S");

    std::time_t timeValue = std::mktime(&timeStruct);
    timeValue += secondsToAdd;
    std::tm* updatedTimeStruct = std::localtime(&timeValue);

    std::ostringstream oss;
    oss << std::put_time(updatedTimeStruct, "%Y-%m-%d %H:%M:%S");

    return oss.str();
}

int main(int argc, const char *argv[])
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " data.csv" << " map.osrm\n";
        return EXIT_FAILURE;
    }

    using namespace osrm;
    auto start = std::chrono::high_resolution_clock::now();
    // Configure based on a .osrm base path, and no datasets in shared mem from osrm-datastore
    EngineConfig config;

    config.storage_config = {argv[2]};
    config.use_shared_memory = false;

    // Use Multilevel Djiskstra algorithm to calculate shortest route
    config.algorithm = EngineConfig::Algorithm::MLD;

    // Routing machine
    const OSRM osrm{config};

    // Parameters for Route querry
    RouteParameters params;
    params.annotations_type = RouteParameters::AnnotationsType::All;

    // Specify the indices of the columns to be extracted (0-based index)
    std::vector<int> selectedColumns = {1, 5, 10, 11, 12, 13};

    std::string outputFolder = "output";
    if (!mkdir(outputFolder.c_str(), 0777))
        std::cout << "Failed to create the output folder." << std::endl;

    std::ifstream input(argv[1]);
    std::ofstream trayectorias(outputFolder + "/trayectorias.txt");
    std::ofstream tiempos(outputFolder + "/tiempos.txt");
    std::ofstream cabeceras(outputFolder + "/cabeceras.csv");

    if (!input.is_open() || !trayectorias.is_open() || !tiempos.is_open() || !cabeceras.is_open()) {
        std::cout << "Error opening files." << std::endl;
        return EXIT_FAILURE;
    }

    std::string line;
    int totalLines = -1;
    while (std::getline(input, line)) {
        totalLines++;
    }
    std::cout << "Processing " << totalLines << " lines.\n";
    input.clear();
    input.seekg(0);

    int currentLine = 0;
    int previousPercentage = -1;

    double lon_o, lat_o, lon_d, lat_d;
    engine::api::ResultT result = json::Object();
    std::getline(input, line); // Discard headers line (comment this line if your csv has no headers)
    while (std::getline(input, line))
    {
        // Update the progress percentage
        currentLine++;        
        int progressPercentage = (currentLine * 100) / totalLines;

        // Print the progress percentage if it has changed
        if (progressPercentage != previousPercentage) {
            std::cout << "\rProgress: " << progressPercentage << "%" << std::flush;
            previousPercentage = progressPercentage;
        }
        
        std::vector<std::string> columns = splitString(line, ','); 
        lon_o = std::stod(columns[10]);
        lat_o = std::stod(columns[11]);
        lon_d = std::stod(columns[12]);
        lat_d = std::stod(columns[13]);
        
        // Discard faulty coordinates
        if (lon_o == 0) continue;

        params.coordinates.push_back({util::FloatLongitude{lon_o}, util::FloatLatitude{lat_o}});
        params.coordinates.push_back({util::FloatLongitude{lon_d}, util::FloatLatitude{lat_d}});

        // Execute routing request, this does the heavy lifting
        const auto status = osrm.Route(params, result);

        auto &json_result = result.get<json::Object>();
        if (status == Status::Error) {
            std::cout << "No routes found for 1 pair of coordinates.\n";
            return EXIT_FAILURE;
        }
        
        auto &routes = json_result.values["routes"].get<json::Array>();
        if (!routes.values.empty()) {
            auto &route = routes.values.at(0).get<json::Object>();
            auto &legs = route.values["legs"].get<json::Array>();
            auto &legObj = legs.values.at(0).get<json::Object>();
            auto &annotation = legObj.values.at("annotation").get<json::Object>();
            auto &nodes = annotation.values.at("nodes").get<json::Array>();
            auto &durations = annotation.values.at("duration").get<json::Array>();
            float sumdur = 0;
            auto durationIt = durations.values.begin();
            auto it = nodes.values.begin();

            trayectorias << std::fixed << static_cast<std::int64_t>((*it).get<json::Number>().value) << " ";
            it++;
            tiempos << 0 << " ";
            
            // Node IDs and Durations
            for (it; it != nodes.values.end(); it++) {
                trayectorias << std::fixed << static_cast<std::int64_t>((*it).get<json::Number>().value) << " ";

                sumdur += (*durationIt).get<json::Number>().value;
                tiempos << sumdur << " ";

                durationIt++;
            }
            trayectorias << 0 << " ";
            tiempos << 'F' << " ";
            
            // Headers
            std::string newLine;
            for (int columnIndex : selectedColumns) {
                if (columnIndex < columns.size()) {
                    newLine += columns[columnIndex] + ',';
                }
            }
            newLine.pop_back();
            newLine.pop_back();
            newLine += ',' + addSecondsToDate(columns[5], sumdur);

            cabeceras << newLine << std::endl;

        }
    }
    std::cout << std::endl;

    input.close();
    trayectorias.close();
    tiempos.close();
    cabeceras.close();

    auto end = std::chrono::high_resolution_clock::now();

    auto time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Execution time: " << time << " milliseconds." << std::endl;


    return EXIT_SUCCESS;
}