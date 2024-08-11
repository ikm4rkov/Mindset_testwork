//
//  DataExporter.hpp
//
//  Created by Cedric Leblond Menard on 16-06-27.
//  Copyright © 2016 Cedric Leblond Menard. All rights reserved.
//

#ifndef DataExporter_hpp
#define DataExporter_hpp

#include <stdio.h>
#include <string>
#include "opencv2/core/core.hpp"
#include <fstream>
#include <iomanip>


// Determine endianness of platform
#define IS_BIG_ENDIAN (!*(unsigned char *)&(uint16_t){1})

enum FileFormat {PLY_ASCII, PLY_BIN_BIGEND, PLY_BIN_LITEND};

class DataExporter  {
private:
    std::string filename = "";
    FileFormat format;
    cv::Mat data;
    cv::Mat colors;
    std::ofstream filestream;
    unsigned long numElem;
    
public:
    DataExporter(cv::Mat outputData, cv::Mat outputColor, std::string outputfile, FileFormat outputformat);
    ~DataExporter();
    void exportToFile();
    
};

#endif /* DataExporter_hpp */
