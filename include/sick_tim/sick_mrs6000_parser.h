/*
 * Copyright (C) 2017, Osnabrück University
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
 *     * Neither the name of Osnabrück University nor the names of its
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
 *
 *      Author:
 *         Michael Ruhnke <michael@marble.io>
 *         Sebastian Pütz <spuetz@uos.de>
 *         Jochen Sprickerhof <jochen@sprickerhof.de>
 *         Martin Günther <mguenthe@uos.de>
 *
 */


#ifndef SICK_MRS6000_PARSER_H_
#define SICK_MRS6000_PARSER_H_

#include "sick_tim/scan_and_cloud_parser.h"
#include <sensor_msgs/point_cloud2_iterator.h>

namespace sick_tim
{

class SickMRS6000Parser : public ScanAndCloudParser
{
 public:
  SickMRS6000Parser();
  virtual ~SickMRS6000Parser();

  virtual int parse_datagram(char* datagram, size_t datagram_length, SickTimConfig &config,
                             sensor_msgs::LaserScan &scan, sensor_msgs::PointCloud2& cloud);

  void set_range_min(float min);
  void set_range_max(float max);
  void set_time_increment(float time);

  sensor_msgs::PointCloud2 cloud_;
  sensor_msgs::PointCloud2Modifier modifier_;
  sensor_msgs::PointCloud2Iterator<float> x_iter, y_iter, z_iter, i_iter;

 private:
  size_t point_counter_;
  int layer_count_;

  float override_range_min_, override_range_max_;
  float override_time_increment_;
};

} /* namespace sick_tim */
#endif /* SICK_MRS1000_PARSER_H_ */
