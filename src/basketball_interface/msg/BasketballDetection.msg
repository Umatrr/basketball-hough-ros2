std_msgs/Header header

bool detected

float32 center_x
float32 center_y
float32 radius

int32 x
int32 y
int32 width
int32 height

float32 confidence

# 中间评分：用于 debug 模式定位颜色或圆周边缘造成的误差。
float32 edge_score
float32 color_score
