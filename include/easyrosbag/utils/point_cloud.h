
#ifndef _POINT_CLOUD_H_
#define _POINT_CLOUD_H_

struct PointXYZIRT {
    float x;
    float y;
    float z;
    double intensity;
    uint8_t ring;
    double timestamp;
};

typedef PointXYZIRT PointType;

#endif //_POINT_CLOUD_H_