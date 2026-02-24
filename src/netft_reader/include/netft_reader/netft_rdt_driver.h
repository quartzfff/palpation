/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

 #ifndef NETFT_RDT_DRIVER
 #define NETFT_RDT_DRIVER
 
 #include <boost/asio.hpp>
 #include <boost/thread/condition.hpp>
 #include <boost/thread/mutex.hpp>
 #include <boost/thread/thread.hpp>
 #include <rclcpp/rclcpp.hpp>
 #include <string>
 
 #include <geometry_msgs/msg/wrench_stamped.hpp>
 #include <std_msgs/msg/bool.hpp>
 
 namespace netft_rdt_driver
 {
 
 class NetFTRDTDriver
 {
 public:
   // Constructor and destructor
   explicit NetFTRDTDriver(const std::string & address, const std::string & frame_id = "base_link");
   ~NetFTRDTDriver();
 
   // Get the most recent data
   void getData(geometry_msgs::msg::WrenchStamped & data);
 
   // Wait for new data (up to 100ms)
   bool waitForNewData(void);
 
   // Zero current wrench as bias
   void zero();
 
 protected:
   void recvThreadFunc(void);
   void startStreaming(void);
   bool readCalibrationInformation(const std::string & address);
 
   enum
   {
     RDT_PORT = 49152,
     TCP_PORT = 49151
   };
 
   std::string address_;
   std::string frame_id_;
 
   boost::asio::io_service io_service_;
   boost::asio::ip::udp::socket socket_;
   boost::mutex mutex_;
   boost::thread recv_thread_;
   boost::condition condition_;
   volatile bool stop_recv_thread_;
   bool recv_thread_running_;
   std::string recv_thread_error_msg_;
 
   geometry_msgs::msg::WrenchStamped new_data_;
 
   unsigned packet_count_;
   unsigned lost_packets_;
   unsigned out_of_order_count_;
   unsigned seq_counter_;
 
   double force_scale_;
   double torque_scale_;
 
   unsigned diag_packet_count_;
   rclcpp::Time last_diag_pub_time_;
 
   uint32_t last_rdt_sequence_;
   uint32_t system_status_;
 
   // ✅ New members for zeroing
   geometry_msgs::msg::Wrench bias_;
   bool bias_set_ = false;
 };
 
 }  // namespace netft_rdt_driver
 
 #endif  // NETFT_RDT_DRIVER
 