#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pybind11/stl.h>
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>

namespace py = pybind11;

namespace pybind11
{
  namespace detail
  {
    template <>
    struct type_caster<cv::Point>
    {

      //! 定义 cv::Point 类型名为 tuple_xy, 并声明类型为 cv::Point 的局部变量 value。
      PYBIND11_TYPE_CASTER(cv::Point, _("tuple_xy"));

      //! 步骤1：从 Python 转换到 C++。
      //! 将 Python tuple 对象转换为 C++ cv::Point 类型, 转换失败则返回 false。
      //! 其中参数2表示是否隐式类型转换。
      bool load(handle obj, bool)
      {
        // 确保传参是 tuple 类型
        if (!py::isinstance<py::tuple>(obj))
        {
          std::logic_error("Point(x,y) should be a tuple!");
          return false;
        }

        // 从 handle 提取 tuple 对象，确保其长度是2。
        py::tuple pt = reinterpret_borrow<py::tuple>(obj);
        if (pt.size() != 2)
        {
          std::logic_error("Point(x,y) tuple should be size of 2");
          return false;
        }

        //! 将长度为2的 tuple 转换为 cv::Point。
        value = cv::Point(pt[0].cast<int>(), pt[1].cast<int>());
        return true;
      }

      //! 步骤2： 从 C++ 转换到 Python。
      //! 将 C++ cv::Mat 对象转换到 tuple，参数2和参数3常忽略。
      static handle cast(const cv::Point &pt, return_value_policy, handle)
      {
        return py::make_tuple(pt.x, pt.y).release();
      }
    };
  }

}