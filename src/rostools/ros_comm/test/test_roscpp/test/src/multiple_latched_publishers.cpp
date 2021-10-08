/*
 * Copyright (c) 2018, Intermodalics BVBA.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Author: Hans-Joachim Krauch */

/*
 * Verify that every latched publisher sends its last published message to
 * a newly connected subscriber.
 */

#include <chrono>
#include <future>
#include <vector>

#include <gtest/gtest.h>
#include "ros/ros.h"
#include "ros/topic_manager.h"
#include <std_msgs/builtin_bool.h>


TEST(MultipleLatchedPublishers, LatchedPublisherReceiveMultiple)
{
  ros::NodeHandle nh;
  std::vector<ros::Publisher> latched_publishers = {
    nh.advertise<bool>("foo", 10, true),
    nh.advertise<bool>("foo", 10, true),
    nh.advertise<bool>("foo", 10, true),
  };
  std::vector<ros::Publisher> unlatched_publishers = {
    nh.advertise<bool>("foo", 10, false),
    nh.advertise<bool>("foo", 10, false),
  };

  for (auto& pub : latched_publishers) {
    pub.publish(true);
  }
  for (auto& pub : unlatched_publishers) {
    pub.publish(true);
  }

  std::size_t msg_count = 0;
  std::promise<bool> received_promise;
  auto received_future = received_promise.get_future();

  const auto sub = nh.subscribe<std_msgs::Bool>("foo", 10, [&](const std_msgs::Bool::ConstPtr&) {
    if (++msg_count == latched_publishers.size()) {
      received_promise.set_value(true);
    }
  });

  ASSERT_EQ(std::future_status::ready, received_future.wait_for(std::chrono::seconds(1)));
  const auto received = received_future.get();
  EXPECT_TRUE(received);
  EXPECT_EQ(latched_publishers.size(), msg_count);
}

TEST(MultipleLatchedPublishers, TopicManagerIsLatched)
{
  ros::NodeHandle nh;
  std::vector<ros::Publisher> publishers_bar = {
    nh.advertise<bool>("bar", 10, false),
    nh.advertise<bool>("bar", 10, true),
    nh.advertise<bool>("bar", 10, false),
  };
  EXPECT_TRUE(ros::TopicManager::instance()->isLatched("/bar"));

  std::vector<ros::Publisher> publishers_baz = {
    nh.advertise<bool>("baz", 10, false),
    nh.advertise<bool>("baz", 10, false),
  };
  EXPECT_FALSE(ros::TopicManager::instance()->isLatched("/baz"));
}

TEST(MultipleLatchedPublishers, TopicManagerLatchShutdown)
{
  ros::NodeHandle nh;
  ros::Publisher qux_latched = nh.advertise<bool>("qux", 10, true);
  ros::Publisher qux_unlatched = nh.advertise<bool>("qux", 10, false);
  EXPECT_TRUE(ros::TopicManager::instance()->isLatched("/qux"));

  qux_latched.shutdown();
  EXPECT_FALSE(ros::TopicManager::instance()->isLatched("/qux"));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_multiple_latched_publishers");
  ros::AsyncSpinner spinner(1);
  spinner.start();
  return RUN_ALL_TESTS();
}

