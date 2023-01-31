/*********************************************************************************************************************
Copyright (c) 2020 RoboSense
All rights reserved

By downloading, copying, installing or using the software you agree to this license. If you do not agree to this
license, do not download, install, copy or use the software.

License Agreement
For RoboSense LiDAR SDK Library
(3-clause BSD License)

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the names of the RoboSense, nor Suteng Innovation Technology, nor the names of other contributors may be used
to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************************************************/

#pragma once

#include <rs_driver/driver/driver_param.hpp>
#include <rs_driver/msg/packet.hpp>
#include <rs_driver/common/error_code.hpp>
#include <rs_driver/macro/version.hpp>
#include <rs_driver/utility/sync_queue.hpp>
#include <rs_driver/utility/buffer.hpp>
#include <rs_driver/driver/input/input_factory.hpp>
#include <rs_driver/driver/decoder/decoder_factory.hpp>

#include <sstream>

namespace robosense
{
namespace lidar
{

inline std::string getDriverVersion()
{
  std::stringstream stream;
  stream << RSLIDAR_VERSION_MAJOR << "."
    << RSLIDAR_VERSION_MINOR << "."
    << RSLIDAR_VERSION_PATCH;

  return stream.str();
}

// 源码讲解 section 4.9
// 类模板，
template <typename T_PointCloud>
class LidarDriverImpl
{
public:
  // 声明构造函数
  LidarDriverImpl();
  // 声明析构函数
  ~LidarDriverImpl();

  // 声明成员函数
  void regPointCloudCallback(

      // 传入& cb_get_cloud: 非空指针
      //
      // 类型 std::function<std::shared_ptr<T_PointCloud>(void)>，这个类型是std::function的返回值
      // std::shared_ptr<T_PointCloud>(void): cb_get_cloud 函数的输入为空，返回类型为 std::shared_ptr<T_PointCloud>
      //
      const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get_cloud,
      const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put_cloud);

  void regPacketCallback(const std::function<void(const Packet&)>& cb_put_pkt);
  void regExceptionCallback(const std::function<void(const Error&)>& cb_excep);
 
  bool init(const RSDriverParam& param);
  bool start();
  void stop();

  void decodePacket(const Packet& pkt);
  bool getTemperature(float& temp);

private:

  void runPacketCallBack(uint8_t* data, size_t data_size, double timestamp, uint8_t is_difop, uint8_t is_frame_begin);
  void runExceptionCallback(const Error& error);

  // 函数 packetGet() 和 packetPut() 用来向 input_ptr_ 注册
  // input_ptr_ 调用前者得到空闲的 Buffer，调用后者派发填充好 Packet 的 Buffer。
  std::shared_ptr<Buffer> packetGet(size_t size);
  void packetPut(std::shared_ptr<Buffer> pkt, bool stuffed);

  void processPacket();

  std::shared_ptr<T_PointCloud> getPointCloud();

  /*
   * 成员函数 splitFrame() 用来向 decoder_ptr_ 注册
   * decoder_ptr_ 在需要分帧时，调用 split_Frame()
   * 这样LidarDriverImpl可以调用cb_put_cloud_将点云传给使用者，
   * 同时调用cb_get_cloud_得到空闲的点云，用于下一帧的累积。
   */
  void splitFrame(uint16_t height, double ts);
  void setPointCloudHeader(std::shared_ptr<T_PointCloud> msg, uint16_t height, double chan_ts);

  RSDriverParam driver_param_;
  // 成员cb_get_cloud_和cb_put_cloud_是回调函数，由驱动的使用者提供。
  // 它们的作用类似于Input类的 cb_get_pkt_ 和 cb_put_pkt_
  // 驱动调用 cb_get_cloud_ 得到空闲的点云，调用 cb_put_cloud_ 派发填充好的点云
  std::function<std::shared_ptr<T_PointCloud>(void)> cb_get_cloud_;
  std::function<void(std::shared_ptr<T_PointCloud>)> cb_put_cloud_;
  std::function<void(const Packet&)> cb_put_pkt_;
  std::function<void(const Error&)> cb_excep_;
  std::function<void(const uint8_t*, size_t)> cb_feed_pkt_;


  std::shared_ptr<Input> input_ptr_;
  std::shared_ptr<Decoder<T_PointCloud>> decoder_ptr_;
  SyncQueue<std::shared_ptr<Buffer>> free_pkt_queue_;
  SyncQueue<std::shared_ptr<Buffer>> pkt_queue_;
  std::thread handle_thread_;
  uint32_t pkt_seq_;
  uint32_t point_cloud_seq_;
  bool to_exit_handle_;
  bool init_flag_;
  bool start_flag_;
};

// 定义构造函数
template <typename T_PointCloud>
inline LidarDriverImpl<T_PointCloud>::LidarDriverImpl()
  : pkt_seq_(0), point_cloud_seq_(0), init_flag_(false), start_flag_(false)
{
}

// 定义析构函数
template <typename T_PointCloud>
inline LidarDriverImpl<T_PointCloud>::~LidarDriverImpl()
{
  stop();
}

// 定义成员函数
template <typename T_PointCloud>
std::shared_ptr<T_PointCloud> LidarDriverImpl<T_PointCloud>::getPointCloud()
{
  /*
   * cb_get_cloud_ 是 rs_driver 的使用者提供的。
   * getPointCloud() 对它加了一层包装，以便较验它是否合乎要求。
   *
   */
  while (1)
  {
    // 调用 cb_get_cloud_ 得到点云
    std::shared_ptr<T_PointCloud> cloud = cb_get_cloud_();
    // 如果点云有效，则将点云大小设置为 0
    if (cloud)
    {
      cloud->points.resize(0);
      return cloud;
    }
    // 如果点云无效，调用 runExceptionCallback() 报告错误
    LIMIT_CALL(runExceptionCallback(Error(ERRCODE_POINTCLOUDNULL)), 1);
  }
}

template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::regPointCloudCallback( 
    const std::function<std::shared_ptr<T_PointCloud>(void)>& cb_get_cloud,
    const std::function<void(std::shared_ptr<T_PointCloud>)>& cb_put_cloud) 
{
  cb_get_cloud_ = cb_get_cloud;
  cb_put_cloud_ = cb_put_cloud;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::regPacketCallback(
    const std::function<void(const Packet&)>& cb_put_pkt)
{
  cb_put_pkt_ = cb_put_pkt;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::regExceptionCallback(
    const std::function<void(const Error&)>& cb_excep)
{
  cb_excep_ = cb_excep;
}


// 初始化 LidarDriverImpl 实例
template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::init(const RSDriverParam& param)
{
  if (init_flag_)
  {
    return true;
  }

  //
  // 初始化 decoder 部分
  //

  // 创建 Decoder 实例
  decoder_ptr_ = DecoderFactory<T_PointCloud>::createDecoder(param.lidar_type, param.decoder_param);
  
  // rewrite pkt timestamp or not ?
  decoder_ptr_->enableWritePktTs((cb_put_pkt_ == nullptr) ? false : true);

  // point cloud related
  decoder_ptr_->point_cloud_ = getPointCloud();  ///< 调用 getPointCloud() 得到空闲点云
  // 调用 Decoder::regCallback(), 传递成员函数 splitFrame()作为参数
  // 这样Decoder分帧时，会调用splitFrame()通知
  decoder_ptr_->regCallback( 
      std::bind(&LidarDriverImpl<T_PointCloud>::runExceptionCallback, this, std::placeholders::_1),
      std::bind(&LidarDriverImpl<T_PointCloud>::splitFrame, this, std::placeholders::_1, std::placeholders::_2));
  // 得到 Decoder 的 Packet 持续时间
  double packet_duration = decoder_ptr_->getPacketDuration();
  bool is_jumbo = isJumbo(param.lidar_type);

  //
  // 初始化 input 部分
  //

  // 创建 Input 实例
  input_ptr_ = InputFactory::createInput(param.input_type, param.input_param, is_jumbo, packet_duration, cb_feed_pkt_);

  // Input组件 收到 packet 后，调用 LidarDriverImpl 的回调函数。回调函数将它保存到 Packet 队列
  // 传递成员函数 packetGet() 和 packetPut()。
  // 这样Input可以得到Buffer， 和派发填充好 Packet 的 Buffer
  input_ptr_->regCallback(
      std::bind(&LidarDriverImpl<T_PointCloud>::runExceptionCallback, this, std::placeholders::_1), 
      std::bind(&LidarDriverImpl<T_PointCloud>::packetGet, this, std::placeholders::_1), 
      std::bind(&LidarDriverImpl<T_PointCloud>::packetPut, this, std::placeholders::_1, std::placeholders::_2));

  // 调用Input::init(), 初始化Input实例。
  if (!input_ptr_->init())
  {
    goto failInputInit;
  }

  driver_param_ = param;
  init_flag_ = true;
  return true;

failInputInit:
  input_ptr_.reset();
  decoder_ptr_.reset();
  return false;
}

// start() 开始处理 MSOP/DIFOP Packet
template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::start()
{
  if (start_flag_)
  {
    return true;
  }

  if (!init_flag_)
  {
    return false;
  }

  // 启动 Packet 处理线程 handle_thread_，线程函数为 processPacket()
  to_exit_handle_ = false;
  handle_thread_ = std::thread(std::bind(&LidarDriverImpl<T_PointCloud>::processPacket, this));

  // 调用 Input::start(), 启动接收线程，接收 MSOP/DIFOP Packet
  input_ptr_->start();

  start_flag_ = true;
  return true;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::stop()
{
  if (!start_flag_)
  {
    return;
  }

  input_ptr_->stop();

  to_exit_handle_ = true;
  handle_thread_.join();

  start_flag_ = false;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::decodePacket(const Packet& pkt)
{
  cb_feed_pkt_(pkt.buf_.data(), pkt.buf_.size());
}

template <typename T_PointCloud>
inline bool LidarDriverImpl<T_PointCloud>::getTemperature(float& temp)
{
  if (decoder_ptr_ == nullptr)
  {
    return false;
  }

  temp = decoder_ptr_->getTemperature();
  return true;
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::runPacketCallBack(uint8_t* data, size_t data_size,
    double timestamp, uint8_t is_difop, uint8_t is_frame_begin)
{
  if (cb_put_pkt_)
  {
    Packet pkt;
    pkt.timestamp = timestamp;
    pkt.is_difop = is_difop;
    pkt.is_frame_begin = is_frame_begin;
    pkt.seq = pkt_seq_++;

    pkt.buf_.resize(data_size);
    memcpy (pkt.buf_.data(), data, data_size);
    cb_put_pkt_(pkt);
  }
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::runExceptionCallback(const Error& error)
{
  if (cb_excep_)
  {
    cb_excep_(error);
  }
}

// packetGet() 分配空闲的 Buffer
template <typename T_PointCloud>
inline std::shared_ptr<Buffer> LidarDriverImpl<T_PointCloud>::packetGet(size_t size)
{
  // 优先从 free_pkt_queue_ 队列得到可用的 Buffer
  std::shared_ptr<Buffer> pkt = free_pkt_queue_.pop();
  // 如果得不到，重新分配一个 Buffer
  if (pkt.get() != NULL)
  {
    return pkt;
  }

  return std::make_shared<Buffer>(size);
}

// packetPut() 将收到的 Packet，放入队列 pkt_queue_
template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::packetPut(std::shared_ptr<Buffer> pkt, bool stuffed)
{
  constexpr static int PACKET_POOL_MAX = 1024;

  if (!stuffed)
  {
    free_pkt_queue_.push(pkt);
    return;
  }

  // 检查 msop_pkt_queue/difop_pkt_queue 中的 Packet 数
  // 如果处理线程太忙，不能及时处理，则释放队列中所有 Buffer
  size_t sz = pkt_queue_.push(pkt);
  if (sz > PACKET_POOL_MAX)
  {
    LIMIT_CALL(runExceptionCallback(Error(ERRCODE_PKTBUFOVERFLOW)), 1);
    pkt_queue_.clear();
  }
}

template <typename T_PointCloud>
inline void LidarDriverImpl<T_PointCloud>::processPacket()
{
  while (!to_exit_handle_)
  {
    // 调用 SyncQueue::popWait() 获得Packet
    std::shared_ptr<Buffer> pkt = pkt_queue_.popWait(500000);
    if (pkt.get() == NULL)
    {
      continue;
    }

    // 检查 Packet 的标志字节
    uint8_t* id = pkt->data();
    if (*id == 0x55)
    {
      // processMsop() 是 MSOP Packet处理线程的函数
      bool pkt_to_split = decoder_ptr_->processMsopPkt(pkt->data(), pkt->dataSize());
      runPacketCallBack(pkt->data(), pkt->dataSize(), decoder_ptr_->prevPktTs(), false, pkt_to_split); // msop packet
    }
    else if (*id == 0xA5)
    {
      decoder_ptr_->processDifopPkt(pkt->data(), pkt->dataSize());
      runPacketCallBack(pkt->data(), pkt->dataSize(), 0, true, false); // difop packet
    }

    // 将Packet的Buffer回收到free_pkt_queue_，等待下次使用
    free_pkt_queue_.push(pkt);
  }
}


// splitFrame()在 Decoder 通知分帧时，派发点云。
template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::splitFrame(uint16_t height, double ts)
{
  // 得到点云
  std::shared_ptr<T_PointCloud> cloud = decoder_ptr_->point_cloud_;
  // 校验点云 point_cloud_
  if (cloud->points.size() > 0)
  {
    // 点云有效，调用 setPointCloudHeader() 设置点云的头部信息
    setPointCloudHeader(cloud, height, ts);
    // 调用 cb_put_cloud_, 将点云传给驱动的使用者
    cb_put_cloud_(cloud);
    // 调用 getPointCloud() 得到空闲点云，重新设置成员 decoder_ptr 的 point_cloud_
    decoder_ptr_->point_cloud_ = getPointCloud();
  }
  else
  {
    runExceptionCallback(Error(ERRCODE_ZEROPOINTS));
  }
}

template <typename T_PointCloud>
void LidarDriverImpl<T_PointCloud>::setPointCloudHeader(std::shared_ptr<T_PointCloud> msg, 
    uint16_t height, double ts)
{
  msg->seq = point_cloud_seq_++;
  msg->timestamp = ts;
  msg->is_dense = driver_param_.decoder_param.dense_points;
  if (msg->is_dense)
  {
    msg->height = 1;
    msg->width = (uint32_t)msg->points.size();
  }
  else
  {
    msg->height = height;
    msg->width = (uint32_t)msg->points.size() / msg->height;
  }
}

}  // namespace lidar
}  // namespace robosense
