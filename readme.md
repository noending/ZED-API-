#ZED介绍
[TOC]
> ZED是一个2K有4MP的RGB感光原件的立体摄像头，2.0的光圈和110度的视度，ZED流用解压WVGA格式视频可达100FPS，3.0USB同时兼容USB2.0.

主要支持4大功能：
  - 视频
  - 深度感知
  - 位置追踪
  - 空间映射

##1.视频
几组支持的视频格式
Video Mode |	Output Resolution (side by side) |	Frame Rate (fps)|	Field of View |
--|--|--|--|
2.2K|	4416x1242|	15|	Wide|
1080p|	3840x1080|	30, 15|	Wide|
720p|	2560x720|	60, 30, 15|	Extra Wide|
WVGA|	1344x376|	100, 60, 30, 15|	Extra Wide|

##1.1 视角选择
- Left view
- Right view
- Side-by-side view
- Left or Right Unrectified
- Left or Right Grayscale

##1.2 调整摄像头设置
可以调整的参数：
- Resolution
- FPS
- Brightness – Controls image brightness.
- Contrast – Controls image contrast.
- Hue – Controls image color.
- Saturation – Controls image color intensity.
- Gamma – Controls gamma correction.
- White Balance – Controls camera white balance.
- Exposure – Controls shutter speed. Setting a long exposure time lead to an increase in blur.
- Gain – Controls digital amplification of the signal from the camera sensor.

选择自动模式，参数会根据场景自动调整
![](https://cdn.stereolabs.com/docs/overview/video/images/explorer_camera_settings.png)

## 1.3视频记录

ZED 是基于UVC的usb相机，所以可以用任何第三方的软件来拍摄MP4或者AVI格式录像，为了更好应用ZED的SDK，视频必须记录成立体的SVO格式，这样ZED的深度，追踪，空间映射API都可以用。

##1.4视频API
ZED API提供对摄像机的底层控制和配置，用ZED之前，需要先创建和打开摄像机对象，API支持两种不同输入，ZED直播模式和ZEDSVD格式的视频文件

#### 1.4.1摄像头配置：
配置摄像头先添加摄像头然后定义InitParameters. 初始化参数可以让你调整相机的分辨率，FPS，深度感知参数等，这些参数只能在开启相机前设置，不能在相机用的是时候改变。

> // 增加一个ZED 相机对象
> Camera zed;
> // 设置配置参数
> InitParameters init_params;
> init_params.camera_resolution = RESOLUTION_HD1080 ;
> init_params.camera_fps = 30 ;

InitParameters包含默认配置，全部参数和函数可以看[这里.](https://www.stereolabs.com/developers/documentation/API/v2.3.0/structsl_1_1InitParameters.html#details)

初始化参数之后，就可以打开相机：
> // Open the camera
> err = zed.open(init_params);
> if (err != SUCCESS)
>    exit(-1);

## 1.4.2视频捕捉
调用grab()来获取新画面,retrieveImage()访问已获取的画面，它可以让你在左和右未修正和灰度图片之间选择.
> sl::Mat image; 
if (zed.grab() == SUCCESS) {
// retrieve the left image 
zed.retrieveImage(image,VIEW_LEFT); 
}

retrieveImage的函数定义看[这里](https://www.stereolabs.com/developers/documentation/API/v2.3.0/classsl_1_1Camera.html#ac40f337ccc76cacd3412b93f7f4638e2),其中第二个参数VIEW可以看[这里](https://www.stereolabs.com/developers/documentation/API/v2.3.0/group__Video__group.html#ga77fc7bfc159040a1e2ffb074a8ad248c)

## 1.4.3 调整相机设置
相机的曝光，白平衡还有其他的一些参数可以在运行中用setCameraSettings()来调整.
> // Set exposure to 50% of camera framerate
zed.setCameraSettings(CAMERA_SETTINGS_EXPOSURE, 50, false);
// Set white balance to 4600K
zed.setCameraSettings(CAMERA_SETTINGS_WHITE_BALANCE, 4600, false);
// Reset to auto exposure
zed.setCameraSettings(CAMERA_SETTINGS_EXPOSURE, -1, true);

用getCameraSettings()来访问当前相机的设定，参看可供使用的设定可以看[这里](https://www.stereolabs.com/developers/documentation/API/v2.3.0/group__Video__group.html#gafdecf198c85568238dacc827f5d085de)


## 1.4.4 获取相机信息
相机的焦距，可视区域还有立体标定以及每个摄像头的分辨率：
> Focal length: fx, fy.
Principal points: cx, cy.
Lens distortion: k1, k2.
Horizontal and vertical field of view.
Stereo calibration: rotation and translation between left and right eye.

这些值都在CalibrationParameters，可以通过getCameraInformation()访问

>CalibrationParameters calibration_params = zed.getCameraInformation()->calibration_parameters;
// Focal length of the left eye in pixels
float focal_left_x = calibration_params.left_cam.fx;
// Horizontal field of view of the left eye in degrees
float h_fov = calibration_params.left_cam.h_fov;

标定的参数可以被相机的自标定技术重新评估，更新的参数在CalibrationParameters中。

## 代码实例
[相机控制](https://github.com/stereolabs/zed-examples/tree/master/camera%20control)和[SVO录像](https://github.com/stereolabs/zed-examples/tree/master/svo%20recording)

## 2深度感知
 ZED基于双筒望远镜的视觉原理，人眼平均瞳据65mm，每只眼睛对周边的看到的画面轻微有所不同，对比两个画面，我们的大脑不仅感知深度同样也能感知在空间中的3D运动，ZED双目相距12cm，捕捉高清场景3D视频来评估深度和运动，通过左右图片之间的像素位移。
 ### 2.1 深度图
 ZED存储图片的（X，Y）像素的距离值（Z）,捕捉的深度图不能直接显示，因为编码是32bit，为了显示深度图需要8-bit的灰度图，值在[0,255]之间，255表示最近的深度值，0表示最远。
 ![](https://cdn.stereolabs.com/docs/overview/depth-sensing/images/zed_depth_standard.jpg)

 ### 3-D点云
 另外一种常见的表示深度信息的方法就是3-D点云，一个点云可以视作深度图的3维信息，深度图只包含距离或者每个像素的Z轴信息，点云是3D（X，Y，Z）的集合，代表外表面场景同时还包含颜色信息。
 ![](https://cdn.stereolabs.com/docs/overview/depth-sensing/images/zed_point_cloud.jpg)

### 2.2 深度感知API
#### 2.2.1 深度感知配置
用InitParameters初始化深度感知，RuntimeParameters在使用中用来改变具体参数

>// Set configuration parameters
InitParameters init_params;
init_params.depth_mode = DEPTH_MODE_ULTRA; // Use ULTRA depth mode
init_params.coordinate_units = UNIT_MILLIMETER; // Use millimeter units (for depth measurements)

更多深度参数配置可以看[这里](https://docs.stereolabs.com/overview/depth-sensing/advanced-settings/)

#### 2.2.2 得到深度数据
用grab()来提取新图片的场景深度图，retrieveMeasure()来访问左图片对应的深度，还可以访问置信图或点云

> sl::Mat image;
sl::Mat depth_map;
if (zed.grab() == SUCCESS) {
  // A new image and depth is available if grab() returns SUCCESS
  zed.retrieveImage(image, VIEW_LEFT); // Retrieve left image
  zed.retrieveMeasure(depth_map, MEASURE_DEPTH); // Retrieve depth
}
深度值默认单位是mm，单位可以通过InitParameters::coordinate_units改变。默认通过cpu内存，或者通过retrieveMeasure(*, *, MEM_GPU)用gpu显存。

#### 2.2.3显示深度图片
用retrieveImage(depth,VIEW_DEPTH)来访文深度图片，不能用8bit的深度图片来显示深度

> sl::Mat depth_for_display;
zed.retrieveImage(depth_for_display,VIEW_DEPTH);

#### 2.2.4 得到点云数据
（X，Y，Z）坐标的3D点云和RGBA颜色和可以通过retrieveMeasure()来读取。

> sl::Mat point_cloud;
zed.retrieveMeasure(point_cloud,MEASURE_XYZRGBA);

用getVaule()的读取确切的像素值。

> float4 point3D;
// Get the 3D point cloud values for pixel (i,j)
point_cloud.getValue(i,j,&point3D);
float x = point3D.x;
float y = point3D.y;
float z = point3D.z;
float color = point3D.w;

用MEASURE_XYZ<COLOR>来选取不同的颜色格式，RGBA颜色可以用retrieveMeasure(point_cloud,MEASURE_XYZBGRA)。

#### 2.2.5 用点云测距
用3D点云取代深度图测距，欧几里得公式可以计算物体到相机左目的相对距离。

> float4 point3D;
// Measure the distance of a point in the scene represented by pixel (i,j)
point_cloud.getValue(i,j,&point3D);
float distance = sqrt(point3D.x*point3D.x + point3D.y*point3D.y + point3D.z*point3D.z);

#### 调整深度分辨率
为了改善你程序的性能和加速数据获取，你可以访问低一点的分辨率，通过定义宽和高的参数在retrieveMeasure()，同样也可以定义数据在CPU（RAM）里访问还是GPU。

> sl::Mat point_cloud;
// Retrieve a resized point cloud
// width and height specify the total number of columns and rows for the point cloud dataset
width = zed.getResolution().width / 2;
height = zed.getResolution().height / 2;
zed.retrieveMeasure(point_cloud, MEASURE_XYZRGBA, MEM_GPU, width, height);

### 2.3 实例代码
可以看[指导](https://github.com/stereolabs/zed-examples/tree/master/tutorials/tutorial%203%20-%20depth%20sensing)和[例子](https://github.com/stereolabs/zed-examples/tree/master/depth%20sensing)


## 3. 位置追踪
位置追踪是设备评估自己和周边世界的相对位置，同样被成为运动追踪，这个就是用带有6度自由（6DoF）的3D空间. ZED用视觉追踪自己的周边和理解用户运动，相机在真是世界中运动，会报告新的位置和方向，这个叫做相机6自由度姿态，姿态信息在相机帧率上输出，最高100FPS在WVGA模式。

### 3.1 得到位置和方向
ZED API可以返回每张图的姿态信息，姿态是通过左目给出，更多可以参考[坐标帧](https://docs.stereolabs.com/overview/positional-tracking/coordinate-frames/),通常世界帧在空间里是固定的。

#### 3.1.2 姿态
姿态数据包括：
- 位置：相机在空间里的位置，通过向量[X,Y,Z]，通常表示总共距离在当前相机位置和对应的坐标帧。
- 方向： 相机在空间里的方向表示为[X,Y,Z,W],可以通过操纵相对的yaw，pitch和roll在不同坐标帧来改变[X,Y,Z,W]

POSE类也同样包含时间戳，置信度和旋转矩阵，这个用来描述相机的旋转和对应的世界帧，这[这里](https://docs.stereolabs.com/overview/positional-tracking/using-tracking/)来了解得到位置和方向的API。

### 3.2 位置追踪API
#### 3.2.1 位置追踪配置
InitParameters初始化，RuntimeParameters改变特定参数在使用中。

> // Set configuration parameters
InitParameters init_params;
init_params.camera_resolution = RESOLUTION_HD720; // Use HD720 video mode (default fps: 60)
init_params.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP; // Use a right-handed Y-up coordinate system
init_params.coordinate_units = UNIT_METER; // Set units in meters

位置追踪用图片和深度信息来评估相机的3D位置，为改善追踪结果，用高FPS视频模式，像HD720和WVGA。

#### 3.2.2 开启位置追踪
打开相机之后，用enableTracking（）和默认参数TrackingParameters开启位置追踪

> // Enable positional tracking with default parameters
sl::TrackingParameters tracking_parameters;
err =  = zed.enableTracking(tracking_parameters);
if (error != SUCCESS)
    exit(-1);

在任何时候都可以关闭追踪用disableTracking()，更多关于位置追踪设置，看[这里](https://www.stereolabs.com/developers/documentation/API/v2.3.0/structsl_1_1TrackingParameters.html)

#### 3.2.3得到姿态

位置会随着新画面而更新，在获取到新的帧之后用getPosition()访问新的姿态，在下面这个例子，我们提取相对于世界帧的姿态和用getTranslation() and getOrientation()来访问转化及方向值.

> sl::Pose zed_pose;
if (zed.grab() == SUCCESS) {
        // Get the pose of the camera relative to the world frame
        TRACKING_STATE state = zed.getPosition(zed_pose, REFERENCE_FRAME_WORLD);
        // Display translation and timestamp
        printf("Translation: tx: %.3f, ty:  %.3f, tz:  %.3f, timestamp: %llu\r",
        zed_pose.getTranslation().tx, zed_pose.getTranslation().ty, zed_pose.getTranslation().tz, zed_pose.timestamp);
        // Display orientation quaternion
        printf("Orientation: ox: %.3f, oy:  %.3f, oz:  %.3f, ow: %.3f\r",
        zed_pose.getOrientation().ox, zed_pose.getOrientation().oy, zed_pose.getOrientation().oz, zed_pose.getOrientation().ow);
    }
}

Pose类用来存相机的位置和时间戳以及置信度，由于位置一直相对参照物，所以把坐标帧设定基础很重要，用REFERENCE_FRAME_WORLD来得到相机的实际位置，或者用REFERENCE_FRAME_CAMERA来得到对上一帧相对位置的改变。
用getTranslation(), getOrientation() and getRotation()来访问转变，方向以及旋转矩阵。

#### 3.2.4 追踪状态
getPosition() 返回一个 TRACKING_STATE，当新的位置出现是，就返回TRACKING_STATE_OK，在初始化或当一个区文件读取时，TRACKING_STATE_SEARCHING 被返回直到相机识别完区域，如果位置追踪帧速率太慢，状态会变为TRACKING_STATE_FPS_TOO_LOW 随后停止返回新的位置。

### 3.3 代码实例
[指导](https://github.com/stereolabs/zed-examples/tree/master/tutorials/tutorial%204%20-%20positional%20tracking)和[例子](https://github.com/stereolabs/zed-examples/tree/master/positional%20tracking)

## 4. 空间映射
空间映射也被成为3D重建，是创建环境的3D地图的能力。它允许设备去理解和与真实世界交互， 空间映射对避免冲撞，运动规划以及现实的真实和虚拟世界的融合非常有用。

ZED持续性的扫描周边和把看到的作成3D地图，地图随着设备的移动和捕捉新的场景元素。因为相机的感知距离超越传统RGB-D感光原件，它可以很快的重建大的室内和室外区域的3D地图。

### 4.1 捕捉一个3D网格
空间映射将真实世界的几何空间映射成一个单一的三角形网格，网格可以持续性的提取来更新，也可以直接一下子映射整个区域。
![](https://cdn.stereolabs.com/docs/overview/spatial-mapping/images/zed_spatial_mapping.jpg)

### 4.2 空间映射参数
空间映射创建三角网格顶点，表面和法线附加到每个顶点。
空间映射的分辨率和范围可以在初始化的时候调整，纹理可以记录最终模型的颜色。

### 4.3 映射分辨率
控制网格的细节层，网格相的数量越高，就能捕获更好的细节。 分辨率可以设置1cm到12cm，捕捉高三角密度的地图需要更多的内存和资源，所以尽可能的在你的程序里用最低的密度。

### 4.4 映射范围
控制深度数据的范围用来构建模型，增加范围可以快速捕捉大量数据但会牺牲准确性，范围可以设置1m到12m，减少范围可以改善空间映射性能

### 4.5 网格过滤

为了改善性能，通常都是减少在捕捉后每个网格的多边形数量，网格的滤波器可以让你毁掉和优化3D模型，在减少多边形的数量同时保存想要的几何特征。

有三个预设，HIGH, MEDIUM and LOW， LOW过滤模式只是简单的填充洞和清理网格立群值，另外两个模式执行网格大毁灭。

### 4.6 网格纹理
网格可以通过投影二维彩色图像在空间映射到三维模型表面，这一步叫做网格纹理填充。

创建纹理，在映射的时候左目图片的子集被记录，每张图像被处理和组装成的单一的纹理映射。这纹理地图然后投射到每个用UV坐标自动生成3D网格面。

### 4.7 API
#### 4.7.1 空间映射配置
用InitParameters来设置视频模式，坐标系统和单位
> // Set configuration parameters
InitParameters init_params;
init_params.camera_resolution = RESOLUTION_HD720; // Use HD720 video mode (default fps: 60)
init_params.coordinate_system = COORDINATE_SYSTEM_RIGHT_HANDED_Y_UP; // Use a right-handed Y-up coordinate system
init_params.coordinate_units = UNIT_METER; // Set units in meters

坐标单位指定网格参数，由于空间映射用位置追踪来作自己的地图，所以为了得到最佳结果推荐用60FPS的HD720的视频

#### 4.7.2 开启空间映射
打开相机后,开启位置跟踪使用enableTracking()，空间映射使用enableSpatialMapping().空间映射的两个主要参数分辨率和范围可以调整。
#### 4.7.3 调整分辨率
映射分辨率可以手动定义（单位：m）也可以通过预先调整：
> MAPPING_RESOLUTION_HIGH: Set resolution to 2cm 设置map较小的区域

> MAPPING_RESOLUTION_MEDIUM: Set resolution to 5cm 在性能和细节方面完美的平衡

>MAPPING_RESOLUTION_LOW: Set resolution of 8cm 一般用于大范围地图区域或创建网格碰撞

高分辨率映射时非常吃资源的同时也降低了网格更新速度，用低分辨率可以更快的更新网格。

>SpatialMappingParameters mapping_parameters;
mapping_parameters.resolution_meter = 0.03 ;  // Set resolution to 3cm
mapping_parameters.resolution_meter = SpatialMappingParameters::get(MAPPING_RESOLUTION_LOW); // Or use preset

#### 4.7.4 调整范围
深度数据的范围在映射过程中可以手动集成（单位m）可以通过以下3个预设：
> MAPPING_RANGE_NEAR: integrates depth up to 3.5 meters.
MAPPING_RANGE_MEDIUM: integrates depth up to 5 meters.
MAPPING_RANGE_FAR: integrates depth up to 10 meters.

由于深度的准确性会随着距离的增加而减少，所以映射范围必须限制在十米左右，选取合适的分辨率和范围来均衡性能和质量的比率

> SpatialMappingParameters mapping_parameters;  
mapping_parameters.range_meter = 5 ;  // Set maximum depth mapping range to 5m
mapping_parameters.range_meter = SpatialMappingParameters::get(MAPPING_RANGE_MEDIUM); // Or use preset

#### 4.7.5 材质
你需要导出网格的材质版本才能通过空间映射到保存场景图像。

> SpatialMappingParameters mapping_parameters;  
mapping_parameters.save_texture = true ;  // Scene texture will be recorded

#### 4.7.5 得到3D地图

启动grab()来开始空间映射，新的图片，深度和追踪将会吸收背景来创建网格

#### 4.7.6 一次生成映射

通过以下方法来处理图片的全部区域和保存结果：

- 开始空间映射，捕捉整个区域
- 完成时，用extractWholeMesh(sl::Mesh)来访问整个网格
- 用mesh.filter()来改善值
- mesh.applyTexture()来创建材质
- 保存网格用mesh.save("filename.obj").

> // Configure spatial mapping parameters
sl::SpatialMappingParameters mapping_parameters(SpatialMappingParameters::MAPPING_RESOLUTION_LOW,SpatialMappingParameters::MAPPING_RANGE_FAR;
mapping_parameters.save_texture = true;
filter_params.set(MeshFilterParameters::MESH_FILTER_LOW);
>
>// Enable tracking and mapping
zed.enableTracking();
zed.enableSpatialMapping(mapping_parameters);
>
>sl::Mesh mesh; // Create a mesh object
int timer=0;
>
>// Grab 500 frames and stop
while (timer < 500) {
  if (zed.grab() == SUCCESS) {
    // When grab() = SUCCESS, a new image, depth and pose is available.
    // Spatial mapping automatically ingests the new data to build the mesh.
    timer++;
  }      
}
>
>// Retrieve the mesh
zed.extractWholeMesh(mesh);
// Filter the mesh
mesh.filter(filter_params);
// Apply the texture
mesh.applyTexture();
// Save the mesh in .obj format
mesh.save("mesh.obj");

#### 4.7.7 连续映射
为持续性得到网格，可以用请求和更新系统：
- 开启空间映射
- 用requestMeshAsync()来发送请求来的个更新后的网格，这会在背景里启动网格提取
- 当getMeshRequestStatusAsync()返回SUCCESS时说明请求状态是网格准备完成
- 用retrieveMeshAsync(sl::Mesh)访问网格
- 应用到你的程序中

请求和访问网格时非常消耗资源的，所以不要频繁的请求新网格

>// Request an updated mesh every 0.5s
sl::Mesh mesh; // Create a mesh object
int timer=0;
while (1) {
  if (zed.grab() == SUCCESS) {

      // Request an update of the mesh every 30 frames (0.5s in HD720 mode)
      if (timer%30 == 0)
         zed.requestMeshAsync();

      // Retrieve mesh when ready
      if (zed.getMeshRequestStatusAsync() == SUCCESS && timer > 0)
         zed.retrieveMeshAsync(mesh);

      timer++;
  }      
}

 #### 4.7.8 关闭空间映射

> // Disable spatial mapping, positional tracking and close the camera
zed.disableSpatialMapping();
zed.disableTracking();
zed.close();
return 0;

#### 4.7.9 空间映射状态

在空间映射过程中推荐用getSpatialMappingState()来检查状态
> SPATIAL_MAPPING_STATE state = zed.getSpatialMappingState();

当空间映射运行没有问题就会返回SPATIAL_MAPPING_STATE_OK，如果返回SPATIAL_MAPPING_STATE_FPS_TOO_LOW就说明帧率过低，如果内存过低就会返回SPATIAL_MAPPING_STATE_NOT_ENOUGH_MEMORY， 以上两种情况，空间映射就会停止整合新的数据到模型中，但3D模型还是可以提取的。

#### 4.8.0 实例代码
[指导](https://github.com/stereolabs/zed-examples/tree/master/tutorials/tutorial%205%20-%20spatial%20mapping)和[例子](https://github.com/stereolabs/zed-examples/tree/master/spatial%20mapping)