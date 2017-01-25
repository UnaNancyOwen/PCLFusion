#include "app.h"
#include "util.h"

#include <thread>
#include <chrono>

#include <ppl.h>
#include <atlbase.h>

// Constructor
Kinect::Kinect()
{
    // Initialize
    initialize();
}

// Destructor
Kinect::~Kinect()
{
    // Finalize
    finalize();
}

// Processing
void Kinect::run()
{
    // Main Loop
    while( !viewer->wasStopped() ){
        // Update Data
        update();

        // Draw Data
        draw();

        // Show Data
        show();

        // Wait a Few Milli Secconds
        std::this_thread::sleep_for( std::chrono::milliseconds( 5 ) );
    }
}

// Initialize
void Kinect::initialize()
{
    // Initialize Sensor
    initializeSensor();

    // Initialize Color
    initializeColor();

    // Initialize Depth
    initializeDepth();

    // Initialize Fusion
    initializeFusion();

    // Initialize Point Cloud
    initializePointCloud();

    // Wait a Few Seconds until begins to Retrieve Data from Sensor ( about 2000-[ms] )
    std::this_thread::sleep_for( std::chrono::seconds( 2 ) );
}

// Initialize Sensor
inline void Kinect::initializeSensor()
{
    // Open Sensor
    ERROR_CHECK( GetDefaultKinectSensor( &kinect ) );

    ERROR_CHECK( kinect->Open() );

    // Check Open
    BOOLEAN isOpen = FALSE;
    ERROR_CHECK( kinect->get_IsOpen( &isOpen ) );
    if( !isOpen ){
        throw std::runtime_error( "failed IKinectSensor::get_IsOpen( &isOpen )" );
    }

    // Retrieve Coordinate Mapper
    ERROR_CHECK( kinect->get_CoordinateMapper( &coordinateMapper ) );
}

// Initialize Color
inline void Kinect::initializeColor()
{
    // Open Color Reader
    ComPtr<IColorFrameSource> colorFrameSource;
    ERROR_CHECK( kinect->get_ColorFrameSource( &colorFrameSource ) );
    ERROR_CHECK( colorFrameSource->OpenReader( &colorFrameReader ) );

    // Retrieve Color Description
    ComPtr<IFrameDescription> colorFrameDescription;
    ERROR_CHECK( colorFrameSource->CreateFrameDescription( ColorImageFormat::ColorImageFormat_Bgra, &colorFrameDescription ) );
    ERROR_CHECK( colorFrameDescription->get_Width( &colorWidth ) ); // 1920
    ERROR_CHECK( colorFrameDescription->get_Height( &colorHeight ) ); // 1080
    ERROR_CHECK( colorFrameDescription->get_BytesPerPixel( &colorBytesPerPixel ) ); // 4

    // Allocation Color Buffer
    colorBuffer.resize( colorWidth * colorHeight * colorBytesPerPixel );
}

// Initialize Depth
inline void Kinect::initializeDepth()
{
    // Open Depth Reader
    ComPtr<IDepthFrameSource> depthFrameSource;
    ERROR_CHECK( kinect->get_DepthFrameSource( &depthFrameSource ) );
    ERROR_CHECK( depthFrameSource->OpenReader( &depthFrameReader ) );

    // Retrieve Depth Description
    ComPtr<IFrameDescription> depthFrameDescription;
    ERROR_CHECK( depthFrameSource->get_FrameDescription( &depthFrameDescription ) );
    ERROR_CHECK( depthFrameDescription->get_Width( &depthWidth ) ); // 512
    ERROR_CHECK( depthFrameDescription->get_Height( &depthHeight ) ); // 424
    ERROR_CHECK( depthFrameDescription->get_BytesPerPixel( &depthBytesPerPixel ) ); // 2

    // Allocation Depth Buffer
    depthBuffer.resize( depthWidth * depthHeight );
}

// Initialize Fusion
inline void Kinect::initializeFusion()
{
    // Set Reconstruction Parameters
    reconstructionParameters.voxelsPerMeter = 256;
    reconstructionParameters.voxelCountX = 512;
    reconstructionParameters.voxelCountY = 384;
    reconstructionParameters.voxelCountZ = 512;

    // Create Reconstruction
    SetIdentityMatrix( worldToCameraTransform );
    ERROR_CHECK( NuiFusionCreateColorReconstruction( &reconstructionParameters, NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE::NUI_FUSION_RECONSTRUCTION_PROCESSOR_TYPE_AMP, -1, &worldToCameraTransform, &reconstruction ) );

    // Set Camera Parameters
    cameraParameters.focalLengthX = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_X;
    cameraParameters.focalLengthY = NUI_KINECT_DEPTH_NORM_FOCAL_LENGTH_Y;
    cameraParameters.principalPointX = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_X;
    cameraParameters.principalPointY = NUI_KINECT_DEPTH_NORM_PRINCIPAL_POINT_Y;

    // Create Image Frame Buffers
    ERROR_CHECK( NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_FLOAT, depthWidth, depthHeight, &cameraParameters, &depthImageFrame ) );
    ERROR_CHECK( NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_FLOAT, depthWidth, depthHeight, &cameraParameters, &smoothDepthImageFrame ) );
    ERROR_CHECK( NuiFusionCreateImageFrame( NUI_FUSION_IMAGE_TYPE::NUI_FUSION_IMAGE_TYPE_COLOR, depthWidth, depthHeight, &cameraParameters, &colorImageFrame ) );
}

// Initialize Point Cloud
inline void Kinect::initializePointCloud()
{
    // Visualizer
    viewer = boost::make_shared<pcl::visualization::PCLVisualizer>( "Point Cloud Viewer" );
    viewer->registerKeyboardCallback( Kinect::keyboardCallback, this );

    // Point Cloud
    cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGBA>>();
    cloud->is_dense = false;
}

// Keyboard Callback Function
void Kinect::keyboardCallback( const pcl::visualization::KeyboardEvent& event, void* cookie )
{
    // Reset Reconstruction
    if( event.getKeySym() == "r" && event.keyDown() ){
        static_cast<Kinect*>( cookie )->reset();
    }
}

// Finalize
void Kinect::finalize()
{
    // Release Image Frame Buffers
    ERROR_CHECK( NuiFusionReleaseImageFrame( depthImageFrame ) );
    ERROR_CHECK( NuiFusionReleaseImageFrame( smoothDepthImageFrame ) );
    ERROR_CHECK( NuiFusionReleaseImageFrame( colorImageFrame ) );

    // Close Sensor
    if( kinect != nullptr ){
        kinect->Close();
    }
}

// Update Data
void Kinect::update()
{
    // Update Color
    updateColor();

    // Update Depth
    updateDepth();

    // Update Fusion
    updateFusion();

    // Update Point Cloud
    updatePointCloud();
}

// Update Color
inline void Kinect::updateColor()
{
    // Retrieve Color Frame
    ComPtr<IColorFrame> colorFrame;
    const HRESULT ret = colorFrameReader->AcquireLatestFrame( &colorFrame );
    if( FAILED( ret ) ){
        return;
    }

    // Convert Format ( YUY2 -> BGRA )
    ERROR_CHECK( colorFrame->CopyConvertedFrameDataToArray( static_cast<UINT>( colorBuffer.size() ), &colorBuffer[0], ColorImageFormat::ColorImageFormat_Bgra ) );
}

// Update Depth
inline void Kinect::updateDepth()
{
    // Retrieve Depth Frame
    ComPtr<IDepthFrame> depthFrame;
    const HRESULT ret = depthFrameReader->AcquireLatestFrame( &depthFrame );
    if( FAILED( ret ) ){
        return;
    }

    // Retrieve Depth Data
    ERROR_CHECK( depthFrame->CopyFrameDataToArray( static_cast<UINT>( depthBuffer.size() ), &depthBuffer[0] ) );
}

// Update Fusion
inline void Kinect::updateFusion()
{
    // Set Depth Data to Depth Float Frame Buffer
    ERROR_CHECK( reconstruction->DepthToDepthFloatFrame( &depthBuffer[0], static_cast<UINT>( depthBuffer.size() * depthBytesPerPixel ), depthImageFrame, NUI_FUSION_DEFAULT_MINIMUM_DEPTH/* 0.5[m] */, NUI_FUSION_DEFAULT_MAXIMUM_DEPTH/* 8.0[m] */, true ) );

    // Smoothing Depth Float Frame
    ERROR_CHECK( reconstruction->SmoothDepthFloatFrame( depthImageFrame, smoothDepthImageFrame, NUI_FUSION_DEFAULT_SMOOTHING_KERNEL_WIDTH, NUI_FUSION_DEFAULT_SMOOTHING_DISTANCE_THRESHOLD ) );

    // Retrieve Mapped Coordinates
    std::vector<ColorSpacePoint> points( depthWidth * depthHeight );
    ERROR_CHECK( coordinateMapper->MapDepthFrameToColorSpace( depthWidth * depthHeight, &depthBuffer[0], depthWidth * depthHeight, &points[0] ) );

    // Mapping Color to Depth Resolution and Set Color Data to Color Frame Buffer
    NUI_FUSION_BUFFER* colorImageFrameBuffer = colorImageFrame->pFrameBuffer;
    RGBQUAD* src = reinterpret_cast<RGBQUAD*>( &colorBuffer[0] );
    RGBQUAD* dst = reinterpret_cast<RGBQUAD*>( colorImageFrameBuffer->pBits );
    Concurrency::parallel_for( 0, depthHeight, [&]( const int y ){
        for( int x = 0; x < depthWidth; x++ ){
            unsigned int index = y * depthWidth + x;
            const ColorSpacePoint point = points[index];
            int colorX = static_cast<int>( point.X + 0.5f );
            int colorY = static_cast<int>( point.Y + 0.5f );
            if( ( 0 <= colorX ) && ( colorX < colorWidth ) && ( 0 <= colorY ) && ( colorY < colorHeight ) ){
                dst[index] = src[colorY * colorWidth + colorX];
            }
            else{
                dst[index] = {};
            }
        }
    } );

    // Retrieve Transformation Matrix to Camera Coordinate System from World Coordinate System
    ERROR_CHECK( reconstruction->GetCurrentWorldToCameraTransform( &worldToCameraTransform ) );

    // Reconstruction Frame Process 
    HRESULT ret = reconstruction->ProcessFrame( smoothDepthImageFrame, colorImageFrame, NUI_FUSION_DEFAULT_ALIGN_ITERATION_COUNT, NUI_FUSION_DEFAULT_INTEGRATION_WEIGHT, NUI_FUSION_DEFAULT_COLOR_INTEGRATION_OF_ALL_ANGLES, nullptr, &worldToCameraTransform );
    if( FAILED( ret ) ){
        // Reset Reconstruction when Retrived Many Accumulated Error Frames ( Over 20 Error Frames )
        static unsigned int errorCount = 0;
        if( ++errorCount >= 20 ){
            errorCount = 0;
            reset();
        }
    }
}

// Update Point Cloud
inline void Kinect::updatePointCloud()
{
    // Calculate Mesh Data
    const UINT step = 3;
    ComPtr<INuiFusionColorMesh> mesh;
    HRESULT ret = reconstruction->CalculateMesh( step, &mesh );
    if( FAILED( ret ) ){
        return;
    }

    // Retrieve Vertex Count
    const unsigned int verticesCount = mesh->VertexCount();
    if( !verticesCount ){
        return;
    }

    // Retrieve Vertices
    const Vector3* vertices = nullptr;
    ERROR_CHECK( mesh->GetVertices( &vertices ) );

    // Retrieve Colors
    const int* colors = nullptr;
    ERROR_CHECK( mesh->GetColors( &colors ) );

    // ReSet Point Cloud
    cloud->clear();
    cloud->width = static_cast<uint32_t>( verticesCount );
    cloud->height = static_cast<uint32_t>( 1 );
    cloud->points.resize( cloud->width * cloud->height );

    // Convert Mesh to Point Cloud
    Concurrency::parallel_for( static_cast<unsigned int>( 0 ), verticesCount, [&]( const unsigned int index ){
        pcl::PointXYZRGBA point;

        const Vector3 vertex = vertices[index];
        point.x = vertex.x;
        point.y = -vertex.y;
        point.z = -vertex.z;

        const uint32_t color = colors[index];
        point.rgba = color;

        cloud->points[index] = point;
    } );
}

// Reset Reconstruction
inline void Kinect::reset()
{
    std::cout << "Reset Reconstruction" << std::endl;

    // Set Identity Matrix
    SetIdentityMatrix( worldToCameraTransform );

    // Reset Reconstruction
    ERROR_CHECK( reconstruction->ResetReconstruction( &worldToCameraTransform, nullptr ) );

    // Clear Point Cloud And Remove Point Cloud in Viewer
    cloud->clear();
    viewer->removeAllPointClouds();
}

// Draw Data
void Kinect::draw()
{
    // Draw Point Cloud
    drawPointCloud();
}

// Draw Point Cloud
inline void Kinect::drawPointCloud()
{
    // Update Point Cloud
    if( !viewer->updatePointCloud( cloud, "cloud" ) ){
        viewer->addPointCloud( cloud, "cloud" );
    }
}

// Show Data
void Kinect::show()
{
    // Show Point Cloud
    showPointCloud();
}

// Show Point Cloud
inline void Kinect::showPointCloud()
{
    // Spin Viewer
    viewer->spinOnce();
}