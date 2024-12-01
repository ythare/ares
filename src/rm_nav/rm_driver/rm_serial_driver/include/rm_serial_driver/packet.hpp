// Copyright (C) 2022 ChenJun
// Copyright (C) 2024 Zheng Yu
// Licensed under the Apache-2.0 License.

#ifndef RM_SERIAL_DRIVER__PACKET_HPP_
#define RM_SERIAL_DRIVER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_serial_driver
{
    struct ReceivePacket
    {
        uint8_t header = 0x55;

        uint8_t decision_num; // decision switch
        // robot hp
        uint16_t red_1_robot_hp;
        uint16_t red_2_robot_hp;
        uint16_t red_3_robot_hp;
        uint16_t red_4_robot_hp;
        uint16_t red_5_robot_hp;
        uint16_t red_7_robot_hp;
        uint16_t red_outpost_hp;
        uint16_t red_base_hp;
        uint16_t blue_1_robot_hp;
        uint16_t blue_2_robot_hp;
        uint16_t blue_3_robot_hp;
        uint16_t blue_4_robot_hp;
        uint16_t blue_5_robot_hp;
        uint16_t blue_7_robot_hp;
        uint16_t blue_outpost_hp;
        uint16_t blue_base_hp;
        // game status
        uint8_t game_progress; // 比赛进程
        uint16_t stage_remain_time;
        // robot status
        uint8_t robot_id;
        uint16_t current_hp;
        uint16_t shooter_heat;
        bool team_color;
        bool is_attacked;
        bool is_detect_enemy;

        uint16_t checksum;

    } __attribute__((packed));

    struct SendPacket
    {
        uint8_t header = 0x55;

        float vx;
        float vy;
        float vz;
        uint16_t checksum;

    } __attribute__((packed));

    inline ReceivePacket fromVector(const std::vector<uint8_t> &data)
    {
        ReceivePacket packet;
        std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
        return packet;
    }

    inline std::vector<uint8_t> toVector(const SendPacket &data)
    {
        std::vector<uint8_t> packet(sizeof(SendPacket));
        std::copy(
            reinterpret_cast<const uint8_t *>(&data),
            reinterpret_cast<const uint8_t *>(&data) + sizeof(SendPacket), packet.begin());
        return packet;
    }

} // namespace rm_serial_driver

#endif // RM_SERIAL_DRIVER__PACKET_HPP_
