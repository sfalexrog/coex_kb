# Использование ROS совместно с CLion

Разработка и отладка ROS-нод требует хороших инструментов, и [CLion](https://www.jetbrains.com/clion/) таковым является: тут тебе и поддержка CMake (который вовсю используется инструментом для ROS-разработки [Catkin](http://wiki.ros.org/catkin)). Но для того, чтобы CLion нормально находил всё необходимое, надо настроить некоторые переменные окружения перед его запуском.

(Вообще, про запуск CLion с ROS есть [статья у JetBrains](https://www.jetbrains.com/help/clion/ros-setup-tutorial.html), и здесь просто кратко рассказано, что надо делать)

Итак, "путь к успеху":

0. Установить ROS - наверное, очевидный шаг, но без этого работать ничего не будет

1. Создать себе рабочее пространство Catkin (можно пропустить, если пространство уже есть):

    ```
    # Это надо выполнять, если пути к ROS не прописаны в .bashrc и/или .profile
    $ source /opt/ros/$ROSDISTRO/setup.bash
    $ mkdir -p catkin_ws/src
    $ cd catkin_ws
    $ catkin_make
    ```

2. Активировать рабочее пространство и запустить CLion:
    
    ```
    # Предполагается, что source /opt/ros/$ROSDISTRO/setup.bash уже выполнен
    # Действия надо выполнять в catkin_ws, созданном в пункте 1
    $ source devel/setup.bash
    # Здесь надо указывать путь до своего CLion
    $ /opt/intellij/clion/bin/clion.sh
    ```

3. Настроить пути для проекта CLion. Делать это надо при загруженном проекте; основной файл проекта - ```catkin_ws/src/CMakeLists.txt```.
   
   В настройках проекта надо открыть ```Build, Execution, Deployment -> CMake```, указать:
   ```
   CMake options: -DCATKIN_DEVEL_PREFIX:PATH=/full/path/to/catkin_ws/devel
   Generation path: /full/path/to/catkin_ws/build
   ```

4. Цель ```Build All``` недоступна, поэтому надо аккуратно собирать все нужные ноды руками. Не очень здорово, но жить можно.

5. Отладку к нодам, запущенным из .launch-файлов, можно подключать через ```Attach to process``` (```ctrl+alt+5```). Запуск происходит так:
   
   ```
   # Легче всего это делать в терминале CLion
   # Синтаксис: roslaunch <имя_пакета> <имя_launch_файла_в_пакете> [аргументы:=паскаль_стайл]
   $ roslaunch clever sitl.launch
   ```
   Затем надо понять, к чему аттачиться:
   ```
   $ rosnode list
   # выдаст список запущеных нод
   ```
   Нужную ноду можно будет выбрать среди процессов, которые есть в меню ```Attach to process```
