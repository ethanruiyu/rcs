.. _coordinate:

坐标系说明
===========

在该系统下，我们将存在三个坐标系：

 - **导航坐标系**
  机器人在定位导航时使用的坐标系，以实际的距离为单位(m)。以建图开始位置为原点(原点可以为任意位置)，使用时可通过yaml文件内容确定原点位置。

  .. image:: images/coordinate-nav.png

  ``nx = ix + (ix * resolution) + origin[0]``

  ``ny = ((height - iy - 1) * resolution) + origin[1]``

 - **图片坐标系**
  导航地图转为png图片时，以图片左上角为原点的坐标系，通常用于路径规划。以像素为单位。

  .. image:: images/coordinate-img.png

  ``ix = int((nx - origin[0]) / resolution)``

  ``iy = int(height - (ny - origin[1]) / resolution - 1)``

 - **画布坐标系**
  在2d可视化界面中，canvas是基于原点进行缩放，所以将canvas原点偏移到图片中心。以像素为单位。

  .. image:: images/coordinate-canvas.png

  ``cx = ix + width / 2``

  ``cy = iy + height / 2``