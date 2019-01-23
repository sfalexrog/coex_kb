# Запуск PX4 с толкача в Gazebo с ROS

Начальные условия: склонирован [PX4](https://github.com/PX4/Firmware), установлен ROS Kinetic,
установлены пакеты ```ros-kinetic-gazebo-ros, ros-kinetic-gazebo-dev, ros-kinetic-gazebo-plugins,
ros-kinetic-gazebo-ros-pkgs, ros-kinetic-gazebo-ros-msgs```, поставлены все пакеты для сборки PX4 (их список можно подсмотреть в [контейнерах для сборки](https://github.com/PX4/containers/blob/master/docker/px4-dev/Dockerfile_base) и [запуска в симуляторе](https://github.com/PX4/containers/blob/master/docker/px4-dev/Dockerfile_simulation))

1. Сборка PX4 и плагинов Gazebo

Используется версия 1.8.2 PX4, для последующих версий названия директорий и целей будут меняться (например, ```posix_sitl``` -> ```px4_sitl```)

В директории ```Firmware``` (куда склонирован репозиторий PX4) пишем

```
$ make posix_sitl_default sitl_gazebo
```

Это не строго обязательно, но ускорит дальнейшие шаги

2. Подготовка окружения к запуску Gazebo

Предполагается, что в текущем окружении уже прописался ROS (то есть уже сделан
```source /opt/ros/kinetic/setup.bash``` - скорее всего, это прописано в ~/.profile или ~/.bashrc)

В директории ```Firmware``` (корне склонированного репозитория PX4) надо выполнить команду

```
$ . Tools/setup_gazebo.bash $(pwd) $(pwd)/build/posix_sitl_default
```

Эта команда пропишет в переменных окружения, где искать собранные библиотеки.

3. Запуск Gazebo с нужной моделью

В директории ```Firmware``` с подготовленным окружением надо выполнить команду

```
$ roslaunch gazebo_ros empty_world.launch world_name:=$(pwd)/Tools/sitl_gazebo/worlds/iris_fpv_cam.world
```

Вместо ```iris_fpv_cam.world``` можно указать любой мир из поставки PX4 (ну или вообще любой другой, на самом деле, надо только разобраться, как их правильно собирать)

4. Запуск PX4

Можно немного упороться и запустить px4 напрямую. Для этого надо выполнить команду

```
$ ./build/posix_sitl_default/px4 -d . posix-configs/SITL/init/ekf2/iris
```

в директории ```Firmware```. При этом ```ekf2``` можно заменить на ```lpe```, ```iris``` - на какое-то из других устройств (это выбор файла инициализации; на самом деле, можно просто посмотреть, что в них прописано).

Альтернатива - использовать магию make-файлов из PX4:

```
$ no_sim=1 make posix_sitl_default gazebo
```

(опять же, из ```Firmware```)

При этом в первом случае можно ненароком лишиться консоли px4, но это же не особо принципиально? В крайнем случае, можно просто взять и подключиться к симулятору через QGroundControl.

5. Бонус - запуск Клевера

Создаём рабочее пространство catkin, собираем пакеты Клевера:

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin_make
$ . devel/setup.bash
$ cd src
$ git clone https://github.com/copterexpress/clever
$ cd ..
$ catkin_make
$ roslaunch clever sitl.launch
```

Теперь должны запуститься ноды Клевера, в каком-нибудь соседнем шелле (в котором настроен ROS и сделан ```source ~/catkin_ws/devel/setup.bash```) можно их дёргать.
