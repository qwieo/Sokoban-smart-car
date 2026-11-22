

#ifndef POSITION_FUSION_H
#define POSITION_FUSION_H
#include <stdint.h>
#define ENCODER_COUNT_PER_PIXEL  120.0f
#define ODOM_SCALE_X  0.900f
#define ODOM_SCALE_Y  0.900f
typedef struct
{
		uint8_t camera_enable;
    /*
     * 融合后的世界坐标位置。
     * 单位：摄像头像素格。
     */
    float x;
    float y;
		float odom_angle;
    /*
     * 正常情况下的摄像头校正权重。
     *
     * 数值越小，坐标越平滑，但摄像头纠偏越慢。
     * 数值越大，跟随摄像头越快，但抖动也越明显。
     */
    float alpha_normal;

    /*
     * 可能发生打滑时的摄像头校正权重。
     */
    float alpha_slip;

    /*
     * 摄像头与预测位置之间的残差超过此值时，
     * 认为可能发生打滑。
     *
     * 单位：像素格。
     */
    float slip_threshold;

    /*
     * 摄像头坐标跳变超过此值时，
     * 认为本帧坐标异常，直接拒绝。
     *
     * 单位：像素格。
     */
    float max_camera_jump;

    /*
     * 连续多少帧摄像头坐标异常后，
     * 允许估计器重新接受摄像头位置。
     *
     * 防止编码器严重打滑后，摄像头一直因为残差过大被拒绝。
     */
    uint8_t max_reject_count;

    uint8_t reject_count;
    uint8_t initialized;
		} PositionFusion_t;

typedef struct
{
    int16_t x_buffer[3];
    int16_t y_buffer[3];

    uint8_t index;
    uint8_t count;

    int32_t smooth_x;
    int32_t smooth_y;
    uint8_t smooth_initialized;
} CameraMedianFilter_t;

void PositionFusion_Init(PositionFusion_t *fusion);

void CameraMedian_Init(CameraMedianFilter_t *filter);

uint8_t CameraMedian_Update(
    CameraMedianFilter_t *filter,
    int16_t camera_x,
    int16_t camera_y,
    int16_t *filtered_x,
    int16_t *filtered_y);


uint8_t PositionFusion_CorrectCamera(
    PositionFusion_t *fusion,
    int16_t camera_x,
    int16_t camera_y);
		
void PositionFusion_UpdateOdometry(
    PositionFusion_t *fusion,
    int16_t cnt1,
    int16_t cnt2,
    int16_t cnt3,
    int16_t cnt4,
    float gyro);
		

#endif