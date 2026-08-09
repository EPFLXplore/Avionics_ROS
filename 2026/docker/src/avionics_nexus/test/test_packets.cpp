/**
 * @file test_packets.cpp
 * @brief Guards the wire contract shared with the firmware: each packed struct
 *        must be exactly the sum of its fields (no padding), and every packet id
 *        must be unique. packets.h already static_asserts the sizes at compile
 *        time; this makes the contract an explicit, runnable check as well.
 *
 * NOTE: these are the *wire* (packed) sizes the two boards memcpy, NOT the ROS
 * CDR-serialized sizes (CDR adds alignment/padding). The wire size is the one
 * that actually has to match across the link.
 */
#include <gtest/gtest.h>

#include "packets.h"

/* Compile-time (redundant with packets.h, but keeps the contract visible here). */
static_assert(sizeof(MassPacket)   == 5, "MassPacket wire size drift");
static_assert(sizeof(Heartbeat)    == 1, "Heartbeat wire size drift");
static_assert(sizeof(ServoRequest) == 11, "ServoRequest wire size drift");
static_assert(sizeof(MassRequest)  == 7, "MassRequest wire size drift");
static_assert(sizeof(LEDRequest)   == 2, "LEDRequest wire size drift");
static_assert(sizeof(PhPacket)     == 4, "PhPacket wire size drift");
static_assert(sizeof(PhRequest)    == 9, "PhRequest wire size drift");

TEST(PacketWireContract, PackedSizes) {
    EXPECT_EQ(sizeof(MassPacket),   5u);
    EXPECT_EQ(sizeof(Heartbeat),    1u);
    EXPECT_EQ(sizeof(ServoRequest), 11u);
    EXPECT_EQ(sizeof(MassRequest),  7u);
    EXPECT_EQ(sizeof(LEDRequest),   2u);
    EXPECT_EQ(sizeof(PhPacket),     4u);
    EXPECT_EQ(sizeof(PhRequest),    9u);
}

TEST(PacketWireContract, DistinctIds) {
    const int ids[] = {MassPacket_ID, Heartbeat_ID, ServoRequest_ID, MassRequest_ID, LEDRequest_ID,
                       PhPacket_ID, PhRequest_ID};
    const size_t n = sizeof(ids) / sizeof(ids[0]);
    for (size_t i = 0; i < n; ++i)
        for (size_t j = i + 1; j < n; ++j)
            EXPECT_NE(ids[i], ids[j]) << "duplicate packet id at " << i << "," << j;
}
