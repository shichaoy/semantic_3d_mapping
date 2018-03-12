/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 */

/**
 * This file has some modifications and bugfixes by Daniel Maturana
 * (dimatura@cmu.edu)
 */

#ifndef POINT_CLOUD2_ITERATOR_H_HVKQJG5A
#define POINT_CLOUD2_ITERATOR_H_HVKQJG5A

#include <sensor_msgs/PointCloud2.h>
#include <cstdarg>
#include <string>
#include <vector>

/**
 * \brief Tools for manipulating sensor_msgs
 *
 * This file provides two sets of utilities to modify and parse PointCloud2
 * The first set allows you to conveniently set the fields by hand:
 * <PRE>
 *   #include <sensor_msgs/point_cloud_iterator.h>
 *   // Create a PointCloud2
 *   sensor_msgs::PointCloud2 cloud_msg;
 *   // Fill some internals of the PoinCloud2 like the header/width/height ...
 *   cloud_msgs.height = 1;  cloud_msgs.width = 4;
 *   // Set the point fields to xyzrgb and resize the vector with the following command
 *   // 4 is for the number of added fields. Each come in triplet: the name of the PointField,
 *   // the number of occurences of the type in the PointField, the type of the PointField
 *   sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
 *   modifier.setPointCloud2FieldsByString(4, "x", 1, sensor_msgs::PointField::FLOAT32,
 *                                            "y", 1, sensor_msgs::PointField::FLOAT32,
 *                                            "z", 1, sensor_msgs::PointField::FLOAT32,
 *                                            "rgb", 1, sensor_msgs::PointField::FLOAT32);
 *   // For convenience and the xyz, rgb, rgba fields, you can also use the following overloaded function.
 *   // You have to be aware that the following function does add extra padding for backward compatibility though
 *   // so it is definitely the solution of choice for PointXYZ and PointXYZRGB
 *   // 2 is for the number of fields to add
 *   modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
 * </PRE>
 *
 * The second set allow you to traverse your PointCloud using an iterator:
 * <PRE>
 *   // Define some raw data we'll put in the PointCloud2
 *   float point_data[] = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0};
 *   uint8_t color_data[] = {40, 80, 120, 160, 200, 240, 20, 40, 60, 80, 100, 120};
 *   // Define the iterators. When doing so, you define the Field you would like to iterate upon and
 *   // the type of you would like returned: it is not necessary the type of the PointField as sometimes
 *   // you pack data in another type (e.g. 3 uchar + 1 uchar for RGB are packed in a float)
 *   sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
 *   sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "y");
 *   sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "z");
 *   // Even though the r,g,b,a fields do not exist (it's usually rgb, rgba), you can create iterators for
 *   // those: they will handle data packing for you (in little endian RGB is packed as *,R,G,B in a float
 *   // and RGBA as A,R,G,B)
 *   sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg, "r");
 *   sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg, "g");
 *   sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg, "b");
 *   // Fill the PointCloud2
 *   for(size_t i=0; i<n_points; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
 *     *iter_x = point_data[3*i+0];
 *     *iter_y = point_data[3*i+1];
 *     *iter_z = point_data[3*i+2];
 *     *iter_r = color_data[3*i+0];
 *     *iter_g = color_data[3*i+1];
 *     *iter_b = color_data[3*i+2];
 *   }
 * </PRE>
 */

namespace ca
{
/**
 * @brief Enables modifying a sensor_msgs::PointCloud2 like a container
 */
class PointCloud2Modifier
{
public:
  /**
   * @brief Default constructor
   * @param cloud_msg The sensor_msgs::PointCloud2 to modify
   */
  PointCloud2Modifier(sensor_msgs::PointCloud2& cloud_msg);

  /**
   * @return the number of T's in the original sensor_msgs::PointCloud2
   */
  size_t size() const;

  /**
   * @param size The number of T's to reserve in the original sensor_msgs::PointCloud2 for
   */
  void reserve(size_t size);

  /**
   * @param size The number of T's to change the size of the original sensor_msgs::PointCloud2 by
   */
  void resize(size_t size);

  /**
   * @brief remove all T's from the original sensor_msgs::PointCloud2
   */
  void clear();

  /**
   * @brief Function setting some fields in a PointCloud and adjusting the
   *        internals of the PointCloud2
   * @param n_fields the number of fields to add. The fields are given as
   *        triplets: name of the field as char*, number of elements in the
   *        field, the datatype of the elements in the field
   *
   * E.g, you create your PointCloud2 message with XYZ/RGB as follows:
   * <PRE>
   *   setPointCloud2Fields(cloud_msg, 4, "x", 1, sensor_msgs::PointField::FLOAT32,
   *                                              "y", 1, sensor_msgs::PointField::FLOAT32,
   *                                              "z", 1, sensor_msgs::PointField::FLOAT32,
   *                                              "rgb", 1, sensor_msgs::PointField::FLOAT32);
   * </PRE>
   * WARNING: THIS DOES NOT TAKE INTO ACCOUNT ANY PADDING AS DONE UNTIL HYDRO
   * For simple usual cases, the overloaded setPointCloud2FieldsByString is what you want.
   */
  void setPointCloud2Fields(int n_fields, ...);

  /**
   * @brief Function setting some fields in a PointCloud and adjusting the
   *        internals of the PointCloud2
   * @param n_fields the number of fields to add. The fields are given as
   *        strings: "xyz" (3 floats), "rgb" (3 uchar stacked in a float),
   *        "rgba" (4 uchar stacked in a float)
   * @return void
   *
   * WARNING: THIS FUNCTION DOES ADD ANY NECESSARY PADDING TRANSPARENTLY
   */
  void setPointCloud2FieldsByString(int n_fields, ...);
protected:
  /** A reference to the original sensor_msgs::PointCloud2 that we read */
  sensor_msgs::PointCloud2& cloud_msg_;
};
}

namespace
{
/** Private base class for PointCloud2Iterator and PointCloud2ConstIterator
 * T is the type of the value to be retrieved
 * U is the type of the raw data in PointCloud2 (only uchar and const uchar are supported)
 */
template<typename T, typename U>
class PointCloud2IteratorBase
{
public:
  /**
   * @param cloud_msg The PointCloud2 to iterate upon
   * @param field_name The field to iterate upon
   */
  void initialize(sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name);

  /** Const version of the above
   * @param cloud_msg The PointCloud2 to iterate upon
   * @param field_name The field to iterate upon
   */
  void initialize(const sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name);

  /** Assignment operator
   * @param iter the iterator to copy data from
   * @return a reference to *this
   */
  PointCloud2IteratorBase<T, U>& operator =(const  PointCloud2IteratorBase<T, U>& iter);

  /** Access the i th element starting at the current pointer (useful when a field has several elements of the same
   * type)
   * @param i
   * @return a reference to the i^th value from the current position
   */
  T& operator [](size_t i) const;

  /** Dereference the iterator. Equivalent to accessing it through [0]
   * @return the value to which the iterator is pointing
   */
  T& operator *() const;

  /** Increase the iterator to the next element
   * @return a reference to the updated iterator
   */
  PointCloud2IteratorBase<T, U>& operator ++();

  /** Basic pointer addition
   * @param i the amount to increase the iterator by
   * @return an iterator with an increased position
   */
  PointCloud2IteratorBase<T, U> operator +(int i);

  /** Increase the iterator by a certain amount
   * @return a reference to the updated iterator
   */
  PointCloud2IteratorBase<T, U>& operator +=(int i);

  /** Compare to another iterator
   * @return whether the current iterator points to a different address than the other one
   */
  bool operator !=(const PointCloud2IteratorBase<T, U>& iter) const;

  /** Return the end iterator
   * @return the end iterator (useful when performing normal iterator processing with ++)
   */
  PointCloud2IteratorBase<T, U> end() const;

private:
  /** Common code to set the field of the PointCloud2
   * @param cloud_msg the PointCloud2 to modify
   * @param field_name the name of the field to iterate upon
   * @return the offset at which the field is found
   */
  int set_field(const sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name);

  /** The "point_step" of the original cloud */
  int point_step_;
  /** The raw data  in uchar* where the iterator is */
  U* data_char_;
  /** The cast data where the iterator is */
  T* data_;
  /** The end() pointer of the iterator */
  T* data_end_;
  /** Whether the fields are stored as bigendian */
  bool is_bigendian_;
};
}

namespace ca
{
/**
 * \brief Class that can iterate over a PointCloud2
 *
 * T type of the element being iterated upon
 * E.g, you create your PointClou2 message as follows:
 * <PRE>
 *   setPointCloud2FieldsByString(cloud_msg, 2, "xyz", "rgb");
 * </PRE>
 *
 * For iterating over XYZ, you do :
 * <PRE>
 *   sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
 * </PRE>
 * and then access X through iter_x[0] or *iter_x
 * You could create an iterator for Y and Z too but as they are consecutive,
 * you can just use iter_x[1] and iter_x[2]
 *
 * For iterating over RGB, you do:
 * <PRE>
 * sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(cloud_msg, "rgb");
 * </PRE>
 * and then access R,G,B through  iter_rgb[0], iter_rgb[1], iter_rgb[2]
 */
// TODO this causes warning both here and upstream ROS.
// TODO check if fixed in future, or fix it myself and submit
// another patch.
template<typename T>
class PointCloud2Iterator : public PointCloud2IteratorBase<T, unsigned char>
{
public:
  PointCloud2Iterator();
  /**
   * @param cloud_msg The PointCloud2 to iterate upon
   * @param field_name The field to iterate upon
   */
  PointCloud2Iterator(sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name);
};

/**
 * \brief Same as a PointCloud2Iterator but for const data
 */
template<typename T>
class PointCloud2ConstIterator : public PointCloud2IteratorBase<const T, const unsigned char>
{
public:
  PointCloud2ConstIterator();
  /**
   * @param cloud_msg The PointCloud2 to iterate upon
   * @param field_name The field to iterate upon
   */
  PointCloud2ConstIterator(const sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name);
};
}

// #include "impl/point_cloud2_iterator.h"

/**
 * \brief Private implementation used by PointCloud2Iterator
 * \author Vincent Rabaud
 */

namespace
{
/** Return the size of a datatype (which is an enum of sensor_msgs::PointField::) in bytes
 * @param datatype one of the enums of sensor_msgs::PointField::
 */
inline int sizeOfPointField(int datatype)
{
  if ((datatype == sensor_msgs::PointField::INT8) || (datatype == sensor_msgs::PointField::UINT8))
    return 1;
  else if ((datatype == sensor_msgs::PointField::INT16) || (datatype == sensor_msgs::PointField::UINT16))
    return 2;
  else if ((datatype == sensor_msgs::PointField::INT32) || (datatype == sensor_msgs::PointField::UINT32) ||
      (datatype == sensor_msgs::PointField::FLOAT32))
    return 4;
  else if (datatype == sensor_msgs::PointField::FLOAT64)
    return 8;
  else
  {
    std::stringstream err;
    err << "PointField of type " << datatype << " does not exist";
    throw std::runtime_error(err.str());
  }
  return -1;
}

/** Private function that adds a PointField to the "fields" member of a PointCloud2
 * @param cloud_msg the PointCloud2 to add a field to
 * @param name the name of the field
 * @param count the number of elements in the PointField
 * @param datatype the datatype of the elements
 * @param offset the offset of that element
 * @return the offset of the next PointField that will be added to the PointCLoud2
 */
inline int addPointField(sensor_msgs::PointCloud2 &cloud_msg, const std::string &name,
                         int count, int datatype, int offset)
{
  sensor_msgs::PointField point_field;
  point_field.name = name;
  point_field.count = count;
  point_field.datatype = datatype;
  point_field.offset = offset;
  cloud_msg.fields.push_back(point_field);

  // Update the offset
  return offset + point_field.count * sizeOfPointField(datatype);
}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace ca
{

inline PointCloud2Modifier::PointCloud2Modifier(sensor_msgs::PointCloud2& cloud_msg)
    : cloud_msg_(cloud_msg)
{
}

inline size_t PointCloud2Modifier::size() const
{
  return cloud_msg_.data.size() / cloud_msg_.point_step;
}

inline void PointCloud2Modifier::reserve(size_t size)
{
  cloud_msg_.data.reserve(size * cloud_msg_.point_step);
}

inline void PointCloud2Modifier::resize(size_t size)
{
  cloud_msg_.data.resize(size * cloud_msg_.point_step);

  // Update height/width
  if (cloud_msg_.height == 1) {
    cloud_msg_.width = size;
    cloud_msg_.row_step = size * cloud_msg_.point_step;
  } else
    if (cloud_msg_.width == 1)
      cloud_msg_.height = size;
    else {
      cloud_msg_.height = 1;
      cloud_msg_.width = size;
      cloud_msg_.row_step = size * cloud_msg_.point_step;
    }
}

inline void PointCloud2Modifier::clear() {
  cloud_msg_.data.clear();

  // Update height/width
  if (cloud_msg_.height == 1) {
    cloud_msg_.row_step = cloud_msg_.width = 0;
  } else {
    if (cloud_msg_.width == 1) {
      cloud_msg_.height = 0;
    } else {
      cloud_msg_.row_step = cloud_msg_.width = cloud_msg_.height = 0;
    }
  }
}


/**
 * @brief Function setting some fields in a PointCloud and adjusting the
 *        internals of the PointCloud2
 * @param n_fields the number of fields to add. The fields are given as
 *        triplets: name of the field as char*, number of elements in the
 *        field, the datatype of the elements in the field
 *
 * E.g, you create your PointCloud2 message with XYZ/RGB as follows:
 * <PRE>
 *   setPointCloud2Fields(cloud_msg, 4, "x", 1, sensor_msgs::PointField::FLOAT32,
 *                                              "y", 1, sensor_msgs::PointField::FLOAT32,
 *                                              "z", 1, sensor_msgs::PointField::FLOAT32,
 *                                              "rgb", 1, sensor_msgs::PointField::FLOAT32);
 * </PRE>
 * WARNING: THIS DOES NOT TAKE INTO ACCOUNT ANY PADDING AS DONE UNTIL HYDRO
 * For simple usual cases, the overloaded setPointCloud2FieldsByString is what you want.
 */
inline void PointCloud2Modifier::setPointCloud2Fields(int n_fields, ...)
{
  cloud_msg_.fields.clear();
  cloud_msg_.fields.reserve(n_fields);
  va_list vl;
  va_start(vl, n_fields);
  int offset = 0;
  for (int i = 0; i < n_fields; ++i) {
    // wtf willow garage?
    // you shouldn't put an order-sensitive macro in an argument list.
    // Create the corresponding PointField
    std::string name(va_arg(vl, char*));
    int count(va_arg(vl, int));
    int datatype(va_arg(vl, int));
    offset = addPointField(cloud_msg_, name, count, datatype, offset);
  }
  va_end(vl);

  // Resize the point cloud accordingly
  cloud_msg_.point_step = offset;
  cloud_msg_.row_step = cloud_msg_.width * cloud_msg_.point_step;
  cloud_msg_.data.resize(cloud_msg_.height * cloud_msg_.row_step);
}

/**
 * @brief Function setting some fields in a PointCloud and adjusting the
 *        internals of the PointCloud2
 * @param n_fields the number of fields to add. The fields are given as
 *        strings: "xyz" (3 floats), "rgb" (3 uchar stacked in a float),
 *        "rgba" (4 uchar stacked in a float)
 * @return void
 *
 * WARNING: THIS FUNCTION DOES ADD ANY NECESSARY PADDING TRANSPARENTLY
 */
inline void PointCloud2Modifier::setPointCloud2FieldsByString(int n_fields, ...)
{
  cloud_msg_.fields.clear();
  cloud_msg_.fields.reserve(n_fields);
  va_list vl;
  va_start(vl, n_fields);
  int offset = 0;
  for (int i = 0; i < n_fields; ++i) {
    // Create the corresponding PointFields
    std::string field_name = std::string(va_arg(vl, char*));
    if (field_name == "xyz") {
      sensor_msgs::PointField point_field;
      // Do x, y and z
      offset = addPointField(cloud_msg_, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
      offset = addPointField(cloud_msg_, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
      offset = addPointField(cloud_msg_, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
      offset += sizeOfPointField(sensor_msgs::PointField::FLOAT32);
    } else {
      if ((field_name == "rgb") || (field_name == "rgba")) {
        offset = addPointField(cloud_msg_, field_name, 1, sensor_msgs::PointField::FLOAT32, offset);
        offset += 3 * sizeOfPointField(sensor_msgs::PointField::FLOAT32);
      } else {
        throw std::runtime_error("Field " + field_name + " does not exist");
      }
    }
  }
  va_end(vl);

  // Resize the point cloud accordingly
  cloud_msg_.point_step = offset;
  cloud_msg_.row_step = cloud_msg_.width * cloud_msg_.point_step;
  cloud_msg_.data.resize(cloud_msg_.height * cloud_msg_.row_step);
}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace
{

/**
 * @param cloud_msg_ The PointCloud2 to iterate upon
 * @param field_name The field to iterate upon
 */
template<typename T, typename U>
void PointCloud2IteratorBase<T, U>::initialize(sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name)
{
  int offset = set_field(cloud_msg, field_name);

  data_char_ = &(cloud_msg.data.front()) + offset;
  data_ = reinterpret_cast<T*>(data_char_);
  data_end_ = reinterpret_cast<T*>(&(cloud_msg.data.back()) + 1 + offset);
}

/** Const version of the above
 * @param cloud_msg The PointCloud2 to iterate upon
 * @param field_name The field to iterate upon
 */
template<typename T, typename U>
void PointCloud2IteratorBase<T, U>::initialize(const sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name)
{
  int offset = set_field(cloud_msg, field_name);

  data_char_ = &(cloud_msg.data.front()) + offset;
  data_ = reinterpret_cast<T*>(data_char_);
  data_end_ = reinterpret_cast<T*>(&(cloud_msg.data.back()) + 1 + offset);
}


/** Assignment operator
 * @param iter the iterator to copy data from
 * @return a reference to *this
 */
template<typename T, typename U>
PointCloud2IteratorBase<T, U>& PointCloud2IteratorBase<T, U>::operator =(const  PointCloud2IteratorBase<T, U> &iter)
{
  if (this != &iter)
  {
    point_step_ = iter.point_step_;
    data_char_ = iter.data_char_;
    data_ = iter.data_;
    data_end_ = iter.data_end_;
    is_bigendian_ = iter.is_bigendian_;
  }

  return *this;
}

/** Access the i th element starting at the current pointer (useful when a field has several elements of the same
 * type)
 * @param i
 * @return a reference to the i^th value from the current position
 */
template<typename T, typename U>
T& PointCloud2IteratorBase<T, U>::operator [](size_t i) const
{
  return *(data_ + i);
}

/** Dereference the iterator. Equivalent to accessing it through [0]
 * @return the value to which the iterator is pointing
 */
template<typename T, typename U>
T& PointCloud2IteratorBase<T, U>::operator *() const
{
  return *data_;
}

/** Increase the iterator to the next element
 * @return a reference to the updated iterator
 */
template<typename T, typename U>
PointCloud2IteratorBase<T, U>& PointCloud2IteratorBase<T, U>::operator ++()
{
  data_char_ += point_step_;
  data_ = reinterpret_cast<T*>(data_char_);
  return *this;
}

/** Basic pointer addition
 * @param i the amount to increase the iterator by
 * @return an iterator with an increased position
 */
template<typename T, typename U>
PointCloud2IteratorBase<T, U> PointCloud2IteratorBase<T, U>::operator +(int i)
{
  PointCloud2IteratorBase<T, U> res = *this;

  res.data_char_ += i*point_step_;
  res.data_ = reinterpret_cast<T*>(data_char_);

  return res;
}

/** Increase the iterator by a certain amount
 * @return a reference to the updated iterator
 */
template<typename T, typename U>
PointCloud2IteratorBase<T, U>& PointCloud2IteratorBase<T, U>::operator +=(int i)
{
  data_char_ += i*point_step_;
  data_ = reinterpret_cast<T*>(data_char_);
  return *this;
}

/** Compare to another iterator
 * @return whether the current iterator points to a different address than the other one
 */
template<typename T, typename U>
bool PointCloud2IteratorBase<T, U>::operator !=(const PointCloud2IteratorBase<T, U>& iter) const
{
  return iter.data_ != data_;
}

/** Return the end iterator
 * @return the end iterator (useful when performing normal iterator processing with ++)
 */
template<typename T, typename U>
PointCloud2IteratorBase<T, U> PointCloud2IteratorBase<T, U>::end() const
{
  PointCloud2IteratorBase<T, U> res;
  res.data_ = data_end_;
  return res;
}

/** Common code to set the field of the PointCloud2
  * @param cloud_msg the PointCloud2 to modify
  * @param field_name the name of the field to iterate upon
  * @return the offset at which the field is found
  */
template<typename T, typename U>
int PointCloud2IteratorBase<T, U>::set_field(const sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name)
{
  is_bigendian_ = cloud_msg.is_bigendian;
  point_step_ = cloud_msg.point_step;
  // make sure the channel is valid
  std::vector<sensor_msgs::PointField>::const_iterator field_iter = cloud_msg.fields.begin(), field_end =
      cloud_msg.fields.end();
  while ((field_iter != field_end) && (field_iter->name != field_name))
    ++field_iter;

  if (field_iter == field_end) {
    // Handle the special case of r,g,b,a (we assume they are understood as the channels of an rgb or rgba field)
    if ((field_name == "r") || (field_name == "g") || (field_name == "b") || (field_name == "a"))
    {
      // Check that rgb or rgba is present
      field_iter = cloud_msg.fields.begin();
      while ((field_iter != field_end) && (field_iter->name != "rgb") && (field_iter->name != "rgba"))
        ++field_iter;
      if (field_iter == field_end)
        throw std::runtime_error("Field " + field_name + " does not exist");
      if (field_name == "r")
      {
        if (is_bigendian_)
          return field_iter->offset + 1;
        else
          return field_iter->offset + 2;
      }
      if (field_name == "g")
      {
        if (is_bigendian_)
          return field_iter->offset + 2;
        else
          return field_iter->offset + 1;
      }
      if (field_name == "b")
      {
        if (is_bigendian_)
          return field_iter->offset + 3;
        else
          return field_iter->offset + 0;
      }
      if (field_name == "a")
      {
        if (is_bigendian_)
          return field_iter->offset + 0;
        else
          return field_iter->offset + 3;
      }
    } else
      throw std::runtime_error("Field " + field_name + " does not exist");
  }

  return field_iter->offset;
}

}

namespace ca
{

template<typename T>
PointCloud2Iterator<T>::PointCloud2Iterator()
{
};

/**
 * @param cloud_msg The PointCloud2 to iterate upon
 * @param field_name The field to iterate upon
 */
template<typename T>
PointCloud2Iterator<T>::PointCloud2Iterator(sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name)
{
  PointCloud2IteratorBase<T, unsigned char>::initialize(cloud_msg, field_name);
};

template<typename T>
PointCloud2ConstIterator<T>::PointCloud2ConstIterator()
{
};

/**
 * @param cloud_msg The PointCloud2 to iterate upon
 * @param field_name The field to iterate upon
 */
template<typename T>
PointCloud2ConstIterator<T>::PointCloud2ConstIterator(const sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name)
{
  PointCloud2IteratorBase<const T, const unsigned char>::initialize(cloud_msg, field_name);
};


}

#endif /* end of include guard: POINT_CLOUD2_ITERATOR_H_HVKQJG5A */
