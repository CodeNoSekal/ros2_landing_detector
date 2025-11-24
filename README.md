# ROS2 Landing Zone Detector

Пакет для определения безопасной зоны посадки дронов с визуализацией облаков точек.

## Структура пакета

- `landing_zone_detector_py/`
  - `publisher.py` — публикует облако точек из NPY.
  - `subscriber.py` — определяет безопасную зону.
  - `config.py` — настраиваемые пути и параметры.
- `launch/`
- `test/`

## Установка

```bash
cd ~/ros2_landing_ws
# Установить зависимости Python
pip install -r requirements.txt
# Построить пакет
colcon build --symlink-install
source install/setup.bash

ros2 run landing_zone_detector publisher
ros2 run landing_zone_detector subscriber
