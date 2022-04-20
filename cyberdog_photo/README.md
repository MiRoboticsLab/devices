# cyberdog_photo
铁蛋拍照模块，由cyberdog_manager调用
## 依赖项
需要OpenCV和protocol包中添加有TakePhoto服务
## 类介绍
### PhotoBase
相机接口类，提供Init初始化函数和TakePhoto拍照函数接口
### PhotoCarpo
相机插件类，实现拍照功能，返回拍照服务结果
## 使用流程
由cyberdog_manager完成以下配置流程：  
1. 通过pluginlib::ClassLoader加载cyberdog::device::PhotoCarpo插件  
2. 调用Init函数初始化，确定是否使用simulator模式  
3. 创建protocol::srv::TakePhoto类型服务器，在服务绑定的回调函数中调用cyberdog::device::PhotoCarpo插件中的TakePhoto函数  
完成以上配置后可以在ROS2中调用此服务，如用命令行：  
`ros2 service call 服务名 protocol/srv/TakePhoto {}`
