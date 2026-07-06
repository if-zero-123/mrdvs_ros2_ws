#include "utils.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

namespace
{
const sensor_msgs::msg::PointField *findField(
    const sensor_msgs::msg::PointCloud2 &msg,
    const std::string &name)
{
    const auto it = std::find_if(msg.fields.begin(), msg.fields.end(),
                                 [&name](const sensor_msgs::msg::PointField &field)
                                 { return field.name == name; });
    return it == msg.fields.end() ? nullptr : &(*it);
}

double readNumericField(
    const uint8_t *point_data,
    const sensor_msgs::msg::PointField &field)
{
    switch (field.datatype)
    {
    case sensor_msgs::msg::PointField::INT8:
        return static_cast<double>(*reinterpret_cast<const int8_t *>(point_data + field.offset));
    case sensor_msgs::msg::PointField::UINT8:
        return static_cast<double>(*reinterpret_cast<const uint8_t *>(point_data + field.offset));
    case sensor_msgs::msg::PointField::INT16:
    {
        int16_t value;
        std::memcpy(&value, point_data + field.offset, sizeof(value));
        return static_cast<double>(value);
    }
    case sensor_msgs::msg::PointField::UINT16:
    {
        uint16_t value;
        std::memcpy(&value, point_data + field.offset, sizeof(value));
        return static_cast<double>(value);
    }
    case sensor_msgs::msg::PointField::INT32:
    {
        int32_t value;
        std::memcpy(&value, point_data + field.offset, sizeof(value));
        return static_cast<double>(value);
    }
    case sensor_msgs::msg::PointField::UINT32:
    {
        uint32_t value;
        std::memcpy(&value, point_data + field.offset, sizeof(value));
        return static_cast<double>(value);
    }
    case sensor_msgs::msg::PointField::FLOAT32:
    {
        float value;
        std::memcpy(&value, point_data + field.offset, sizeof(value));
        return static_cast<double>(value);
    }
    case sensor_msgs::msg::PointField::FLOAT64:
    {
        double value;
        std::memcpy(&value, point_data + field.offset, sizeof(value));
        return value;
    }
    default:
        return std::numeric_limits<double>::quiet_NaN();
    }
}

double toRelativeMilliseconds(double raw_time, double cloud_start_sec)
{
    if (!std::isfinite(raw_time))
        return 0.0;

    const double start_us = cloud_start_sec * 1.0e6;
    const double start_ns = cloud_start_sec * 1.0e9;

    if (std::fabs(raw_time - start_us) < 60.0e6)
        return (raw_time - start_us) / 1.0e3;
    if (std::fabs(raw_time - start_ns) < 60.0e9)
        return (raw_time - start_ns) / 1.0e6;
    if (std::fabs(raw_time - cloud_start_sec) < 60.0)
        return (raw_time - cloud_start_sec) * 1.0e3;

    // MRDVS offset_time is microseconds relative to the frame timebase.
    if (raw_time >= 0.0 && raw_time < 1.0e6)
        return raw_time / 1.0e3;

    return 0.0;
}
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr Utils::pointCloud2ToPCL(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    int filter_num,
    double min_range,
    double max_range)
{
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
    const auto *x_field = findField(*msg, "x");
    const auto *y_field = findField(*msg, "y");
    const auto *z_field = findField(*msg, "z");
    if (!x_field || !y_field || !z_field)
        return cloud;

    const auto *intensity_field = findField(*msg, "intensity");
    const auto *time_field = findField(*msg, "timestamp");
    if (!time_field)
        time_field = findField(*msg, "time");
    if (!time_field)
        time_field = findField(*msg, "offset_time");

    filter_num = std::max(1, filter_num);
    const size_t point_num = static_cast<size_t>(msg->width) * static_cast<size_t>(msg->height);
    cloud->reserve(point_num / static_cast<size_t>(filter_num) + 1);

    const double cloud_start_sec = Utils::getSec(msg->header);
    const double min_range_sq = min_range * min_range;
    const double max_range_sq = max_range * max_range;

    for (size_t row = 0; row < msg->height; ++row)
    {
        for (size_t col = 0; col < msg->width; col += static_cast<size_t>(filter_num))
        {
            const uint8_t *point_data = &msg->data[row * msg->row_step + col * msg->point_step];
            const double x = readNumericField(point_data, *x_field);
            const double y = readNumericField(point_data, *y_field);
            const double z = readNumericField(point_data, *z_field);
            const double range_sq = x * x + y * y + z * z;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
                continue;
            if (range_sq < min_range_sq || range_sq > max_range_sq)
                continue;

            pcl::PointXYZINormal p;
            p.x = static_cast<float>(x);
            p.y = static_cast<float>(y);
            p.z = static_cast<float>(z);
            p.intensity = intensity_field ? static_cast<float>(readNumericField(point_data, *intensity_field)) : 0.0f;

            double relative_ms = 0.0;
            if (time_field)
                relative_ms = toRelativeMilliseconds(readNumericField(point_data, *time_field), cloud_start_sec);
            p.curvature = static_cast<float>(std::max(0.0, relative_ms));
            cloud->push_back(p);
        }
    }

    return cloud;
}

double Utils::getSec(const std_msgs::msg::Header &header)
{
    return static_cast<double>(header.stamp.sec) + static_cast<double>(header.stamp.nanosec) * 1e-9;
}
builtin_interfaces::msg::Time Utils::getTime(const double &sec)
{
    builtin_interfaces::msg::Time time_msg;
    time_msg.sec = static_cast<int32_t>(sec);
    time_msg.nanosec = static_cast<uint32_t>((sec - time_msg.sec) * 1e9);
    return time_msg;
}
