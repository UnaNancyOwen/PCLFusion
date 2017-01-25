#ifndef __APP__
#define __APP__

#define NOMINMAX
#include <Windows.h>
#include <Kinect.h>
#include <NuiKinectFusionApi.h>
// Quote from Kinect for Windows SDK v2.0 - Samples/Native/KinectFusionExplorer-D2D, and Partial Modification
// KinectFusionHelper is: Copyright (c) Microsoft Corporation. All rights reserved.
#include "KinectFusionHelper.h"
#include <pcl/visualization/pcl_visualizer.h>

#include <vector>

#include <wrl/client.h>
using namespace Microsoft::WRL;

class Kinect
{
private:
    // Sensor
    ComPtr<IKinectSensor> kinect;

    // Coordinate Mapper
    ComPtr<ICoordinateMapper> coordinateMapper;

    // Reader
    ComPtr<IColorFrameReader> colorFrameReader;
    ComPtr<IDepthFrameReader> depthFrameReader;

    // Fusion
    ComPtr<INuiFusionColorReconstruction> reconstruction;

    // Color Buffer
    std::vector<BYTE> colorBuffer;
    int colorWidth;
    int colorHeight;
    unsigned int colorBytesPerPixel;

    // Depth Buffer
    std::vector<UINT16> depthBuffer;
    int depthWidth;
    int depthHeight;
    unsigned int depthBytesPerPixel;

    // Fusion Buffer
    NUI_FUSION_IMAGE_FRAME* depthImageFrame;
    NUI_FUSION_IMAGE_FRAME* smoothDepthImageFrame;
    NUI_FUSION_IMAGE_FRAME* colorImageFrame;
    NUI_FUSION_RECONSTRUCTION_PARAMETERS reconstructionParameters;
    NUI_FUSION_CAMERA_PARAMETERS cameraParameters;
    Matrix4 worldToCameraTransform;

    // Point Cloud
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud;

public:
    // Constructor
    Kinect();

    // Destructor
    ~Kinect();

    // Processing
    void run();

private:
    // Initialize
    void initialize();

    // Initialize Sensor
    inline void initializeSensor();

    // Initialize Color
    inline void initializeColor();

    // Initialize Depth
    inline void initializeDepth();

    // Initialize Fusion
    inline void initializeFusion();

    // Initialize Point Cloud
    inline void initializePointCloud();

    // Keyboard Callback Function
    static void keyboardCallback( const pcl::visualization::KeyboardEvent &event, void* cookie );

    // Finalize
    void finalize();

    // Update Data
    void update();

    // Update Color
    inline void updateColor();

    // Update Depth
    inline void updateDepth();

    // Update Fusion
    inline void updateFusion();

    // Update Point Cloud
    inline void updatePointCloud();

    // Reset Reconstruction
    inline void reset();

    // Draw Data
    void draw();

    // Draw Point Cloud
    inline void drawPointCloud();

    // Show Data
    void show();

    // Show Point Cloud
    inline void showPointCloud();
};

#endif // __APP__