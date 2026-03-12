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
```

Otonom modda hedef pose göndermek için örnek:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.40, y: -0.55, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.7071, w: 0.7071}}}"
```

## Simülasyonda Kamera(`sensor_msgs/Image`)

URDF içine kamera sensörü eklemek tek başına ROS2'de `sensor_msgs/Image` üretmez; Gazebo Sim içindeki kamera verisini ROS2'ye **bridge** etmek gerekir.

Kamerayı ROS2 tarafında görmek için:
```bash
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

### Test için kırmızı küre spawn etme

Launch varsayılan olarak statik bir kırmızı küre spawn eder.

Kapatmak veya konumunu değiştirmek için:
```bash
ros2 launch pavlov_description gazebo.launch.py spawn_red_ball:=false
ros2 launch pavlov_description gazebo.launch.py red_ball_x:=2.0 red_ball_y:=0.0 red_ball_z:=0.05
```
