#include "osrm/route_parameters.hpp"
#include "osrm/coordinate.hpp"
#include "osrm/engine_config.hpp"
#include "osrm/json_container.hpp"
#include "osrm/osrm.hpp"

#include <iostream>
#include <string>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <vector>
#include <sys/stat.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstdint>

#define EDATE "2012-12-31 00:00:00" // Earliest trip starting date
#define ID_COLUMN   1               // Column of the object id
#define DATE_COLUMN 5               // Column of the csv for the starting date
#define LONG_O      10              // Column of the longitude of origin
#define LAT_O       11              // Column of the latitude of origin
#define LONG_D      12              // Column of the longitude of destination
#define LAT_D       13              // Column of the latitude of destination
#define MAX_TRAY_SIZE 7516192768    // Max size of output trayectorie files

// Function to split a string based on delimiter
std::vector<std::string> splitString(const std::string& line, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(line);
    std::string token;
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token.empty()?" ":token);
    }
    return tokens;
}

// Returns a time_t with the seconds since the epoc from a string date
std::time_t convertStringToTimePoint(const std::string& dateString) {

    std::tm tm = {};
    std::istringstream iss(dateString);
    iss >> std::get_time(&tm, "%Y-%m-%d %H:%M:%S");

    return std::mktime(&tm);
}

// Add seconds to String Date in format "Y-m-d H:M:S"
std::string addSecondsToDate(const std::string& dateString, int secondsToAdd) {

    std::time_t timeValue = convertStringToTimePoint(dateString);
    timeValue += secondsToAdd;
    std::tm* updatedTimeStruct = std::localtime(&timeValue);
    std::ostringstream oss;
    oss << std::put_time(updatedTimeStruct, "%Y-%m-%d %H:%M:%S");

    return oss.str();
}

// Convert string date to seconds since defined EDATE
int dateToInt(const std::string dateString, std::chrono::_V2::system_clock::time_point edateTP) {

    std::time_t dateTT = convertStringToTimePoint(dateString);
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::from_time_t(dateTT) - edateTP);
    
    return static_cast<int>(seconds.count());
}

std::vector<uint8_t> uint64ToBinary(uint64_t trajectory) {
    std::vector<uint8_t> binary;
    binary.reserve(64); // Reserve space for 64 bits

    for (int i = 63; i >= 0; --i) {
        uint8_t bit = (trajectory >> i) & 1;
        binary.push_back(bit);
    }

    return binary;
}

// Function to get the size of a file in bytes
std::uintmax_t getFileSize(const std::string& filename) {
    struct stat fileStat;
    if (stat(filename.c_str(), &fileStat) == 0) {
        return fileStat.st_size;
    }
    return 0;
}

int updateProgress(int curLine, int totalLines, int prevPercentage) {
    int progressPercentage = (curLine * 100) / totalLines;
    // Print the progress percentage if it has changed
    if (progressPercentage != prevPercentage) {
        std::cout << "\rProgress: " << progressPercentage << "%  " << std::flush;
        prevPercentage = progressPercentage;
    }
    return prevPercentage;
}

int main(int argc, const char *argv[])
{
    if (argc < 4)
    {
        std::cerr << "Usage: " << argv[0] << " data.csv" << " map.osrm" << " /outputdir" << std::endl;
        return EXIT_FAILURE;
    }

    using namespace osrm;
    auto start = std::chrono::high_resolution_clock::now();
    // Configure based on a .osrm base path
    EngineConfig config;

    config.storage_config = {argv[2]};
    config.use_shared_memory = false;

    // Use Multilevel Dijkstra algorithm to calculate shortest route
    config.algorithm = EngineConfig::Algorithm::MLD;

    // Routing machine
    const OSRM osrm{config};

    // Parameters for Route query
    RouteParameters params;
    params.annotations_type = RouteParameters::AnnotationsType::All;

    // Specify the indexes of the columns to be extracted (0-based index)
    std::vector<int> selectedColumns = {ID_COLUMN, LONG_O, LAT_O, LONG_D, LAT_D};

    std::string outputFolder = argv[3] + std::string("/output");
    struct stat buffer{};
    if (stat(outputFolder.c_str(), &buffer))
        mkdir(outputFolder.c_str(), 0777);

    std::ifstream input(argv[1]);
    std::ofstream trayec_rlz(outputFolder + "/trayectorias-rlz", std::ios::binary);
    std::ofstream tiempos(outputFolder + "/tiempos.txt");
    std::ofstream cabeceras(outputFolder + "/cabeceras.csv");
    std::ofstream errores(outputFolder + "/errores.txt");

    if(!input.is_open() || !trayec_rlz.is_open() || !tiempos.is_open() || !cabeceras.is_open() || !errores.is_open()){
        std::cout << "Error opening files." << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "Counting lines to track progress..." << std::endl;
    std::string line;
    uint64_t totalLines = -1; // discard header
    while (std::getline(input, line)) {
        totalLines++;
    }
    std::cout << "Processing " << totalLines << " lines..." << std::endl;
    input.clear();
    input.seekg(0);

    uint64_t currentLine = 0;
    int previousPercentage = -1;

    time_t edateTT = convertStringToTimePoint(EDATE);

    double lon_o, lat_o, lon_d, lat_d;
    engine::api::ResultT result = json::Object();

    std::getline(input, line); // Headers line
    std::vector<std::string> headers = splitString(line, ',');
    std::string headersLine;

    // Headers for output csv
    for (int columnIndex : selectedColumns) {
        if (columnIndex < headers.size()) {
            headersLine += headers[columnIndex] + ',';
        }
    }
    headersLine.pop_back();
    headersLine.pop_back();

    headersLine += ',' + std::string("pickup_datetime") + ',' + std::string("dropoff_datetime");
    cabeceras << headersLine << std::endl;

    int faultyLines = 0;
    uint64_t nodenumber = 0;
    char separator = ' ';
    std::vector<uint8_t> tray_sep = uint64ToBinary(0);
    const std::uintmax_t maxSize = MAX_TRAY_SIZE;
    while (std::getline(input, line))
    {
        // Update the progress percentage
        currentLine++;
        previousPercentage = updateProgress(currentLine, totalLines, previousPercentage);

        params.coordinates.clear();
        std::vector<std::string> columns = splitString(line, ',');
        if (columns[LONG_O]==" "|columns[LAT_O]==" "|columns[LONG_D]==" "|columns[LAT_D]==" "|columns[ID_COLUMN]==" "|columns[DATE_COLUMN]==" ") {
            faultyLines++;
            errores<<"Line: "<<currentLine<<" | Reason: at least one null mandatory column |"<<" Contents: "<<line<<std::endl;
            continue;
        }
        lon_o = std::stod(columns[LONG_O]);
        lat_o = std::stod(columns[LAT_O]);
        lon_d = std::stod(columns[LONG_D]);
        lat_d = std::stod(columns[LAT_D]);

        params.coordinates.emplace_back(util::FloatLongitude{lon_o}, util::FloatLatitude{lat_o});
        params.coordinates.emplace_back(util::FloatLongitude{lon_d}, util::FloatLatitude{lat_d});

        // Execute routing request, this does the heavy lifting
        const auto status = osrm.Route(params, result);

        auto &json_result = result.get<json::Object>();
        if (status == Status::Error) {
            faultyLines++;
            errores<<"Line: "<<currentLine<<" | Reason: OSRM couldn't trace route |"<<" Contents: "<<line<<std::endl;
            continue;
        }
        
        auto &routes = json_result.values["routes"].get<json::Array>();
        if (!routes.values.empty()) {
            auto &route = routes.values.at(0).get<json::Object>();
            auto &legs = route.values["legs"].get<json::Array>();
            auto &legObj = legs.values.at(0).get<json::Object>();
            auto &annotation = legObj.values.at("annotation").get<json::Object>();
            auto &nodes = annotation.values.at("nodes").get<json::Array>();
            auto &durations = annotation.values.at("duration").get<json::Array>();
            int sumdur = 0;
            auto durationIt = durations.values.begin();
            auto it = nodes.values.begin();

            //trayectorias << static_cast<std::uint64_t>(((*it).get<json::Number>().value)) << " ";
            std::vector<uint8_t> binaryTrajectory = uint64ToBinary((*it).get<json::Number>().value);
            trayec_rlz.write(reinterpret_cast<char*>(binaryTrajectory.data()), binaryTrajectory.size());
            trayec_rlz.write(&separator, sizeof(separator));
            it++;
            nodenumber++;
            tiempos << 0 << " ";

            // Node IDs and Durations
            for (it; it != nodes.values.end(); it++) {
                std::vector<uint8_t> binaryTrajectory = uint64ToBinary((*it).get<json::Number>().value);
                trayec_rlz.write(reinterpret_cast<char*>(binaryTrajectory.data()), binaryTrajectory.size());
                trayec_rlz.write(&separator, sizeof(separator));                nodenumber++;
                sumdur += static_cast<int>(std::round((*durationIt).get<json::Number>().value));
                tiempos << sumdur << " ";
                durationIt++;
            }
            //trayectorias << 0 << " ";
            trayec_rlz.write(reinterpret_cast<char*>(tray_sep.data()), tray_sep.size());
            tiempos << UINT32_MAX << " ";
            
            // Headers
            std::string newLine;
            for (int columnIndex : selectedColumns)
                if (columnIndex < columns.size())
                    newLine += columns[columnIndex] + ',';
            newLine.pop_back();
            newLine.pop_back();
            newLine += ',' + std::to_string(static_cast<std::uint32_t>(dateToInt(columns[DATE_COLUMN], std::chrono::system_clock::from_time_t(edateTT))));
            newLine += ',' + std::to_string(static_cast<std::uint32_t>(dateToInt(addSecondsToDate(columns[DATE_COLUMN], sumdur), std::chrono::system_clock::from_time_t(edateTT))));
            cabeceras << newLine << std::endl;

            std::uintmax_t currentSize = getFileSize(outputFolder + "/trayectorias-rlz");
            if (currentSize > maxSize) {
                std::cout << "RLZ Trajectory file reached max established file size" << std::endl;
                break;
            }

        }
    }
    std::cout << std::endl;

    input.close();
    trayec_rlz.close();
    tiempos.close();
    cabeceras.close();
    errores.close();

    // Print execution time
    auto end = std::chrono::high_resolution_clock::now();
    auto time = end - start;
    auto hours = std::chrono::duration_cast<std::chrono::hours>(time);
    time -= hours;
    auto minutes = std::chrono::duration_cast<std::chrono::minutes>(time);
    time -= minutes;
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(time);
    std::cout << "Execution time: ";
    std::cout << std::setfill('0');
    std::cout << std::setw(2) << hours.count() << "H:";
    std::cout << std::setw(2) << minutes.count() << "M:";
    std::cout << std::setw(2) << seconds.count() << "S" << std::endl;

    std::cout << "Lines with errors: " << faultyLines << " (stored in " << outputFolder + "/errores.txt" << ")";
    std::cout << std::endl;
    std::cout << "Total number of nodes in " << outputFolder + "/trayec_rlz.txt: " << nodenumber << std::endl;

    return EXIT_SUCCESS;
}