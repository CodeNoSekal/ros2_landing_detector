# ROS2 Landing Zone Detector

Пакет для определения безопасной зоны посадки дронов с визуализацией облаков точек.

---

## Поддерживаемые версии

- ROS2 Humble 
- Python 3.10+

---

## Структура пакета

- `landing_zone_detector_py/`
  - `publisher.py` — публикует облако точек из NPY-файла в ROS2 топик `lidar_points`.
  - `subscriber.py` — подписывается на топик `lidar_points`, ищет безопасную зону посадки и выводит координаты и угол наклона.
  - `config.py` — настраиваемые пути и параметры.
- `launch/` 
- `test/`

---

## Установка

```bash
# Создаём рабочее пространство ROS2 (если его нет)
mkdir -p ~/ros2_landing_ws/src
cd ~/ros2_landing_ws/src

# Клонируем репозиторий
git clone https://github.com/CodeNoSekal/ros2_landing_detector.git

# Установка зависимостей Python
cd ros2_landing_detector
pip install -r requirements.txt

# Сборка пакета
cd ~/ros2_landing_ws
colcon build --symlink-install

# Активируем окружение
source install/setup.bash

```

## Запуск
```bash
#В одной консоли
source ~/ros2_landing_ws/install/setup.bash
ros2 run landing_zone_detector publisher

#Во второй консоли
source ~/ros2_landing_ws/install/setup.bash
ros2 run landing_zone_detector subscriber
```

## Визуализация
В этом репозитории лежит vis.py файл. Он уже подготовлен для запуска. В нём нужно только указать номер патча N, путь к папке и вставить полученные из subscriber координаты в том виде, как они были выведены.
После первого получение координат программу можно прервать.


Сжатые облака точек доступны по этой ссылке
https://cloud.mirea.ru/index.php/s/9rgGJYma4cFwXpP

Архив с полными и со сжатыми облаками точек доступен по этой ссылке
https://cloud.mirea.ru/index.php/s/4rnRYznr9iNgA7j
