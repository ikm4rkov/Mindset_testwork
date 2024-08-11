// требует opencv2 и DataExporter.hpp, первым аргументом принимает на вход файл с списком путей до кадров, вторым - фокусное расстояние, третьим и четвертым - разрешение матрицы фотоаппарата (?)
// код реконструкции сцены из https://docs.opencv.org/3.1.0/d4/d18/tutorial_sfm_scene_reconstruction.html
#include <opencv2/sfm.hpp>
#include <opencv2/viz.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;
using namespace cv;
using namespace cv::sfm;
static void help() {
  cout
      << "\n------------------------------------------------------------------------------------\n"
      << " This program shows the multiview reconstruction capabilities in the \n"
      << " OpenCV Structure From Motion (SFM) module.\n"
      << " It reconstruct a scene from a set of 2D images \n"
      << " Usage:\n"
      << "        example_sfm_scene_reconstruction <path_to_file> <f> <cx> <cy>\n"
      << " where: path_to_file is the file absolute path into your system which contains\n"
      << "        the list of images to use for reconstruction. \n"
      << "        f  is the focal lenght in pixels. \n"
      << "        cx is the image principal point x coordinates in pixels. \n"
      << "        cy is the image principal point y coordinates in pixels. \n"
      << "------------------------------------------------------------------------------------\n\n"
      << endl;
}
// код экспорта модели из https://gist.github.com/cedriclmenard/9088b1ce70c27b1493eec14e82ef28b1
//  DataExporter.cpp
//
//  Created by Cedric Leblond Menard on 16-06-27.
//  Copyright © 2016 Cedric Leblond Menard. All rights reserved.
//

#include "DataExporter.hpp"

// Dictionary for file format
const char* enum2str[] = {
    "ascii",
    "binary_big_endian",
    "binary_little_endian"
};

// Function to reverse float endianness
float ReverseFloat( const float inFloat )
{
    float retVal;
    char *floatToConvert = ( char* ) & inFloat;
    char *returnFloat = ( char* ) & retVal;

    // swap the bytes into a temporary buffer
    returnFloat[0] = floatToConvert[3];
    returnFloat[1] = floatToConvert[2];
    returnFloat[2] = floatToConvert[1];
    returnFloat[3] = floatToConvert[0];

    return retVal;
}

// Check system endianness
bool isLittleEndian()
{
    uint16_t number = 0x1;
    char *numPtr = (char*)&number;
    return (numPtr[0] == 1);
}

DataExporter::DataExporter(cv::Mat outputData, cv::Mat outputColor, std::string outputfile, FileFormat outputformat) :
filename(outputfile), format(outputformat), data(outputData), colors(outputColor)
{
    // MARK: Init
    // Opening filestream
    switch (format) {
        case FileFormat::PLY_BIN_LITEND:
        case FileFormat::PLY_BIN_BIGEND:
            filestream.open(filename, std::ios::out | std::ios::binary);
            break;
        case FileFormat::PLY_ASCII:
            filestream.open(filename,std::ios::out);
            break;
    }

    // Calculating number of elements
    CV_Assert(data.total() == colors.total());      // If not same size, assert
    //CV_Assert(data.type() == CV_32FC3 &&
    //          colors.type() == CV_8UC3);            // If not 3 channels and good type
    CV_Assert(data.isContinuous() &&
              colors.isContinuous());               // If not continuous in memory

    numElem = data.total();

}

void DataExporter::exportToFile() {
    // MARK: Header writing
    filestream << "ply" << std::endl <<
    "format " << enum2str[format] << " 1.0" << std::endl <<
    "comment file created using code by Cedric Menard" << std::endl <<
    "element vertex " << numElem << std::endl <<
    "property float x" << std::endl <<
    "property float y" << std::endl <<
    "property float z" << std::endl <<
    "property uchar red" << std::endl <<
    "property uchar green" << std::endl <<
    "property uchar blue" << std::endl <<
    "end_header" << std::endl;

    // MARK: Data writing

    // Pointer to data
    const float* pData = data.ptr<float>(0);
    const unsigned char* pColor = colors.ptr<unsigned char>(0);
    const unsigned long numIter = 3*numElem;                            // Number of iteration (3 channels * numElem)
    const bool hostIsLittleEndian = isLittleEndian();

    float_t bufferXYZ;                                                 // Coordinate buffer for float type

    // Output format switch
    switch (format) {
        case FileFormat::PLY_BIN_BIGEND:
            // Looping through all
            for (unsigned long i = 0; i<numIter; i+=3) {                                // Loop through all elements
                for (unsigned int j = 0; j<3; j++) {                                    // Loop through 3 coordinates
                    if (hostIsLittleEndian) {
                        bufferXYZ = ReverseFloat(pData[i+j]);                        // Convert from host to network (Big endian)
                        filestream.write(reinterpret_cast<const char *>(&bufferXYZ),    // Non compiled cast to char array
                                         sizeof(bufferXYZ));
                    } else {
                        bufferXYZ = pData[i+j];
                        filestream.write(reinterpret_cast<const char *>(&bufferXYZ),    // Non compiled cast to char array
                                         sizeof(bufferXYZ));
                    }
                }
                for (int j = 2; j>=0; j--) {
                    // OpenCV uses BGR format, so the order of writing is reverse to comply with the RGB format
                    filestream.put(pColor[i+j]);                                        // Loop through RGB
                }
            }

            break;

        case FileFormat::PLY_BIN_LITEND:                                                // Assume host as little-endian
            for (unsigned long i = 0; i<numIter; i+=3) {                                // Loop through all elements
                for (unsigned int j = 0; j<3; j++) {                                    // Loop through 3 coordinates
                    if (hostIsLittleEndian) {
                        filestream.write(reinterpret_cast<const char *>(pData+i+j),     // Non compiled cast to char array
                                         sizeof(bufferXYZ));
                    } else {
                        bufferXYZ = ReverseFloat(pData[i+j]);
                        filestream.write(reinterpret_cast<const char *>(&bufferXYZ), sizeof(bufferXYZ));
                    }
                }
                for (int j = 2; j>=0; j--) {
                    // OpenCV uses BGR format, so the order of writing is reverse to comply with the RGB format
                    filestream.put(pColor[i+j]);                                        // Loop through RGB
                }
            }

            break;

        case FileFormat::PLY_ASCII:
            for (unsigned long i = 0; i<numIter; i+=3) {                            // Loop through all elements
                for (unsigned int j = 0; j<3; j++) {                                // Loop through 3 coordinates
                    filestream << std::setprecision(9) << pData[i+j] << " ";
                }
                for (int j = 2; j>=0; j--) {
                    // OpenCV uses BGR format, so the order of writing is reverse to comply with the RGB format
                    filestream << (unsigned short)pColor[i+j] << (j==0?"":" ");                     // Loop through RGB
                }
                filestream << std::endl;                                            // End if element line
            }
            break;

        default:
            break;
    }
}

DataExporter::~DataExporter() {
    filestream.close();
}
int getdir(const string _filename, vector<string> &files)
{
  ifstream myfile(_filename.c_str());
  if (!myfile.is_open()) {
    cout << "Unable to read file: " << _filename << endl;
    exit(0);
  } else {;
    size_t found = _filename.find_last_of("/\\");
    string line_str, path_to_file = _filename.substr(0, found);
    while ( getline(myfile, line_str) )
      files.push_back(path_to_file+string("/")+line_str);
  }
  return 1;
}
int main(int argc, char* argv[])
{
  // Read input parameters
  if ( argc != 5 )
  {
    help();
    exit(0);
  }
  // Parse the image paths
  vector<string> images_paths;
  getdir( argv[1], images_paths );
  // Build instrinsics
  float f  = atof(argv[2]),
        cx = atof(argv[3]), cy = atof(argv[4]);
  Matx33d K = Matx33d( f, 0, cx,
                       0, f, cy,
                       0, 0,  1);
  bool is_projective = true;
  vector<Mat> Rs_est, ts_est, points3d_estimated;
  reconstruct (images_paths, Rs_est, ts_est, K, points3d_estimated, is_projective);
  // Print output
  cout << "\n----------------------------\n" << endl;
  cout << "Reconstruction: " << endl;
  cout << "============================" << endl;
  cout << "Estimated 3D points: " << points3d_estimated.size() << endl;
  cout << "Estimated cameras: " << Rs_est.size() << endl;
  cout << "Refined intrinsics: " << endl << K << endl << endl;
  cout << "3D Visualization: " << endl;
  cout << "============================" << endl;
  viz::Viz3d window("Coordinate Frame");
             window.setWindowSize(Size(500,500));
             window.setWindowPosition(Point(150,150));
             window.setBackgroundColor(); // black by default
  // Create the pointcloud
  cout << "Recovering points  ... ";
  // recover estimated points3d
  vector<Vec3f> point_cloud_est;
  for (int i = 0; i < points3d_estimated.size(); ++i)
    point_cloud_est.push_back(Vec3f(points3d_estimated[i]));
  cout << "[DONE]" << endl;
  cout << "Recovering cameras ... ";
  vector<Affine3d> path;
  for (size_t i = 0; i < Rs_est.size(); ++i)
    path.push_back(Affine3d(Rs_est[i],ts_est[i]));
  cout << "[DONE]" << endl;
  cv::Mat xyzMap;
  xyzMap = cv::Mat(point_cloud_est, true);
  if ( point_cloud_est.size() > 0 )
  {
    cout << "Rendering points   ... ";
    viz::WCloud cloud_widget(point_cloud_est, viz::Color::green());
    window.showWidget("point_cloud", cloud_widget);
    cout << "[DONE]" << endl;
    DataExporter data(xyzMap, xyzMap, "site.ply", FileFormat::PLY_ASCII);
    data.exportToFile();
  }
  else
  {
    cout << "Cannot render points: Empty pointcloud" << endl;
  }
  if ( path.size() > 0 )
  {
    cout << "Rendering Cameras  ... ";
    window.showWidget("cameras_frames_and_lines", viz::WTrajectory(path, viz::WTrajectory::BOTH, 0.1, viz::Color::green()));
    window.showWidget("cameras_frustums", viz::WTrajectoryFrustums(path, K, 0.1, viz::Color::yellow()));
    window.setViewerPose(path[0]);
    cout << "[DONE]" << endl;
  }
  else
  {
    cout << "Cannot render the cameras: Empty path" << endl;
  }
  cout << endl << "Press 'q' to close each windows ... " << endl;
  window.spin();
  return 0;
}
