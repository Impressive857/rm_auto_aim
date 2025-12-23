std_msgs/Header header
bool is_detected	#是否检测到，1：任意一种检测设备检测到 0:任何一种设备都没有检测到
bool occlusion		#是否遮挡1：全向感知未识别；0：全向感知识别
bool invincible		#是否无敌
bool main		#主相机是否检测到1 or 0
string id		#机器人id 1，2，3，4，sentry，outpost，base
int64 hp		#机器人血量
geometry_msgs/Pose pose #敌方机器人的位置
float32 attack_enhance  #等效易伤情况
