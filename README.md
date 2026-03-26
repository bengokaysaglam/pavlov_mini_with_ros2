# Pavlov Mini – ROS2 Quadruped Robot

Bu repo, **ROS2 tabanlı bir quadruped** robotun simülasyon + kontrol yazılımını içerir.

## Özellikler
- ROS2 node tabanlı kontrol mimarisi
- (Simülasyonda) kamera ile görsel test altyapısı
- Görüntü işleme(openCV) ile object detect

## Sistem Mimarisi (özet)
- **High-level control:** Raspberry Pi (ROS2)
- **Low-level control:** Teensy microcontroller
- **İletişim:** Serial / ROS2 topics
- **Kamera:** Raspberry Pi Camera V2

## Hızlı Başlangıç (Simülasyon)

```bash
colcon build --symlink-install
source install/setup.bash
```

Gazebo Sim + robotu başlat:
```bash
ros2 launch pavlov_description gazebo.launch.py
```

Klavye ile manuel kontrol (teleop):
```bash
ros2 launch pavlov_control pavlov_autonomy.launch.py initial_mode:=teleop
ros2 run pavlov_control keyboard_teleop.py

Otonom modda hedef pose göndermek için örnek:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.40, y: -0.55, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}}}"
```

## Simülasyonda Kamera(`sensor_msgs/Image`)

Kamerayı ROS2 tarafında görmek için:
```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

## Kameradan top konumu (monocular)

`pavlov_control/ball_localizer.py` node'u, görüntüdeki kırmızı küreyi HSV threshold ile bulur ve **ekrandaki piksel yarıçapından mesafeyi** tahmin edip topun 3D konumunu yayınlar.

Topic’leri kontrol et:
```bash
ros2 topic echo /ball/point
ros2 topic echo /ball/pose
ros2 topic echo /ball/visible
ros2 run rqt_image_view rqt_image_view /ball/debug_image
```

Algılama ayarı (top uzakta kayboluyorsa):
```bash
ros2 launch pavlov_control ball_localizer.launch.py sat_min:=60 val_min:=50 min_radius_px:=3.0 max_range_m:=10.0 kernel_size:=3
```

Akış:
1) `ball_localizer` topun `base_link` frame’inde konumunu yayınlar (`/ball/point`, `/ball/pose`).
2) `ball_to_goal` bu konumu `/goal_pose` olarak (frame=`base_link`) yayınlar.
3) `go_to_goal` `header.frame_id=base_link` gelen goal’leri **relative goal** olarak yorumlar.
