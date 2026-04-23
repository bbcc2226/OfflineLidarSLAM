#ifndef OFFLINE_LIDAR_SLAM_TOOLS_KEYFRAME_JSON_IO_HPP_
#define OFFLINE_LIDAR_SLAM_TOOLS_KEYFRAME_JSON_IO_HPP_

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <unordered_map>

#include <DataType.hpp>
#include "TicToc.hpp"
#include "Tools/KeyFrameJsonRecord.hpp"

namespace tools {

inline std::string EscapeJsonString(const std::string& input) {
    std::ostringstream out;
    for (const char ch : input) {
        switch (ch) {
            case '\\':
                out << "\\\\";
                break;
            case '"':
                out << "\\\"";
                break;
            case '\n':
                out << "\\n";
                break;
            case '\r':
                out << "\\r";
                break;
            case '\t':
                out << "\\t";
                break;
            default:
                out << ch;
                break;
        }
    }
    return out.str();
}

inline std::string Vec3ToJson(const Vec3& vec) {
    std::ostringstream out;
    out << '[' << vec.x() << ", " << vec.y() << ", " << vec.z() << ']';
    return out.str();
}

inline std::string QuaternionToJson(const Eigen::Quaterniond& q) {
    std::ostringstream out;
    out << '[' << q.x() << ", " << q.y() << ", " << q.z() << ", " << q.w() << ']';
    return out.str();
}

inline std::string PoseToJson(const Se3& pose) {
    std::ostringstream out;
    const Eigen::Quaterniond q(pose.rotationMatrix());
    out << "{\n"
        << "      \"translation\": " << Vec3ToJson(pose.translation()) << ",\n"
        << "      \"quaternion_xyzw\": " << QuaternionToJson(q) << "\n"
        << "    }";
    return out.str();
}

inline std::string KeyFrameRecordToJsonLine(const KeyFrameJsonRecord& record) {
    std::ostringstream out;
    out << "{" << '\n'
        << "  \"key_frame_id\": " << record.key_frame_id << ",\n"
        << "  \"timestamp\": " << record.timestamp << ",\n"
        << "  \"saved_frame_path\": \"" << EscapeJsonString(record.saved_frame_path) << "\",\n"
        << "  \"valid_rtk\": " << (record.valid_rtk ? "true" : "false") << ",\n"
        << "  \"rtk_pos\": " << Vec3ToJson(record.rtk_pos) << ",\n"
        << "  \"rtk_align_yaw\": " << record.rtk_align_yaw << ",\n"
        << "  \"lio_pose\": " << PoseToJson(record.lio_pose) << ",\n"
        << "  \"optimized_pose\": " << PoseToJson(record.optimized_pose) << '\n'
        << "}";
    return out.str();
}

inline std::string ResolveKeyFrameJsonPath(
    const std::unordered_map<int, std::shared_ptr<KeyFrame>>& key_frame_info_map,
    const std::string& keyframe_json_output_path) {
    (void)key_frame_info_map;
    if (!keyframe_json_output_path.empty()) {
        return keyframe_json_output_path;
    }
    return (std::filesystem::current_path() / "key_frames.jsonl").string();
}

inline void TruncateKeyFrameJsonl(const std::string& output_path) {
    std::ofstream json_file(output_path, std::ios::trunc);
    if (!json_file.is_open()) {
        std::cerr << "Failed to truncate keyframe JSONL file: " << output_path << "\n";
    }
}

inline void AppendKeyFrameRecordToJsonl(
    const KeyFrameJsonRecord& record,
    const std::string& output_path) {
    std::ofstream json_file(output_path, std::ios::app);
    if (!json_file.is_open()) {
        std::cerr << "Failed to open keyframe JSONL file for writing: " << output_path << "\n";
        return;
    }

    json_file << KeyFrameRecordToJsonLine(record) << '\n';
}

inline void RecordKeyFrameToJson(
    const int key_frame_id,
    const Se3& optimized_pose,
    const std::unordered_map<int, std::shared_ptr<KeyFrame>>& key_frame_info_map,
    std::string& keyframe_json_output_path) {
    TicToc timer;
    timer.tic();

    const auto info_it = key_frame_info_map.find(key_frame_id);
    if (info_it == key_frame_info_map.end()) {
        std::cerr << "Key frame info not found for frame " << key_frame_id
                  << " when recording JSON.\n";
        return;
    }

    const auto& key_frame_info = info_it->second;
    if (keyframe_json_output_path.empty()) {
        keyframe_json_output_path = ResolveKeyFrameJsonPath(key_frame_info_map, keyframe_json_output_path);
        TruncateKeyFrameJsonl(keyframe_json_output_path);
    }

    KeyFrameJsonRecord record;
    record.key_frame_id = key_frame_info->key_frame_id_;
    record.timestamp = key_frame_info->timestamp_;
    record.saved_frame_path = key_frame_info->saved_frame_path_;
    record.valid_rtk = key_frame_info->valid_rtk_;
    record.rtk_pos = key_frame_info->rtk_pos_;
    record.rtk_align_yaw = key_frame_info->rtk_align_yaw_;
    record.lio_pose = key_frame_info->lio_pose_;
    record.optimized_pose = optimized_pose;

    AppendKeyFrameRecordToJsonl(record, keyframe_json_output_path);
    timer.toc("Key frame JSONL recording time is");
}

}  // namespace tools

#endif
