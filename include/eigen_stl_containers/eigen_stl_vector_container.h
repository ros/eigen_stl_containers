/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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
 *     with the distribution.
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

/* Author: Ioan Sucan */

#ifndef EIGEN_STL_CONTAINERS_EIGEN_STL_VECTOR_CONTAINER_
#define EIGEN_STL_CONTAINERS_EIGEN_STL_VECTOR_CONTAINER_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include <vector>

/** \brief Typedef's for STL containers of Eigen types with proper memory alignment. */
namespace EigenSTL
{

template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

using vector_Vector3f = AlignedVector<Eigen::Vector3f>;
using vector_Vector3d = AlignedVector<Eigen::Vector3d>;
using vector_Vector4f = AlignedVector<Eigen::Vector4f>;
using vector_Vector4d = AlignedVector<Eigen::Vector4d>;
using vector_Affine3f = AlignedVector<Eigen::Affine3f>;
using vector_Affine3d = AlignedVector<Eigen::Affine3d>;
using vector_Isometry3f = AlignedVector<Eigen::Isometry3f>;
using vector_Isometry3d = AlignedVector<Eigen::Isometry3d>;

}

#endif
