# Сборка ROS-пакетов в deb-пакеты (на примере OpenCV3)

Основная инструкция, по которой я всё делал, [находится здесь](https://gist.github.com/awesomebytes/196eab972a94dd8fcdd69adfe3bd1152). От меня разве что небольшие дополнения.

Требования: ROS-пакет должен быть достаточно оформлен (должен присутствовать ```package.xml```) либо должен быть уже опубликован в ROS (именно так было в случае с ```opencv3```).

Сборка производилась в уже собранном образе Клевера; при этом скрипты сборки должны быть немного изменены:

 - В ```builder/image-build.sh``` стоит увеличить размер образа (```max '7G'```->```max '10G'```, хотя может получиться и с меньшим объёмом)
- В ```builder/image-build.sh``` стоит также убрать или закомментировать последнюю строчку (```${BUILDER_DIR}/image-resize.sh ${IMAGE_PATH}```) - за счёт этого образ останется нужного размера
- Возможно, есть и более удобные способы "входа" в образ, но в данный момент сборщик ожидает, что запускаться будут только скрипты; поэтому в директорию ```builder``` стоит положить файлик ```run_bash.sh``` со следующим содержимым:
    ```
    #!/bin/bash
    /bin/bash
    ```

## Вход в образ

После сборки образа (в репозитории Клевера выполнить ```docker run --privileged --rm -v /dev:/dev -v $(pwd):/builder/repo goldarte/img-tool:builder-mod```) надо зайти в docker-контейнер сборщика:

```
$ docker run --privileged --rm -it -v /dev:/dev -v $(pwd):/builder/repo goldearte/img-tool:builder-mod /bin/bash
```

Внутри контейнера надо выполнить (вместо ```GIT_HASH``` следует подставить суффикс образа Клевера):

```
# # Рабочая папка должна быть /builder
# ./image-chroot.sh /builder/repo/image/clever_${GIT_HASH}.img exec /builder/repo/builder/run_bash.sh 
```

Если ошибок не было, то в терминале появится приглашение ```bash``` из образа.

## Требуемые пакеты

Требуется ROS-инструментарий (по крайней мере, ```catkin```). Помимо этого, потребуются:

 - ```bloom``` (ставится как ```sudo apt-get install python-bloom``` или ```sudo pip install -U bloom```) - утилита для генерации необходимых файлов конфигурирования Debian

 - ```fakeroot``` (```sudo apt-get install fakeroot```) - для сборки из исходников

 - ```debhelper``` и ```dpkg-dev``` (```sudo apt-get install debhelper dpkg-dev```) для пакования пакета

При этом надо помнить, что в запущенном окружении ```sudo``` работает не очень хорошо (из root в pi зайти получится, а вот обратно - не очень). Поэтому просто от root'а можно выполнить:

```
# apt-get install python-bloom fakeroot debhelper dpkg-dev
```

Теперь - собственно сборка:

0. Собирать надо от пользователя ```pi```; можно "залогиниться" от его имени командой

    ```
    # sudo -u pi /bin/bash -l
    $ cd /home/pi
    ```

1. Обновить список пакетов!

    ```
    $ rosdep update
    ```

2. Создать рабочую папку и в ней подгрузить всё необходимое для сборки пакета

    ```
    $ mkdir ros_catkin_ws
    $ cd ros_catkin_ws
    $ rosinstall_generator opencv3 --rosdistro kinetic > kinetic-opencv3.rosinstall
    $ wstool init -j8 src kinetic-opencv3.rosinstall
    ```

3. Зайти в ```src/opencv3``` (проверить наличие файла ```package.xml```!) и подготовить его к сборке:
   
    ```
    $ bloom-generate rosdebian --os-name debian --os-version stretch --ros-distro kinetic
    ```

4. Появится папка ```debian```. Флаги компиляции и параметры сборки можно поправить в файле ```debian/rules```. Разумно поставить следующее:

   - ```export DEB_BUILD_OPTIONS="parallel=8"``` среди экспортов
   - ```dh $@ -v --buildsystem=cmake --parallel``` - добавить --parallel, что должно заставить сборку собираться многопоточно (но я не увидел эффекта от этого параметра)
   - в ```override_dh_auto_configure``` можно (и нужно!) прописать флаги сборки
   - в ```override_dh_auto_build``` можно добавить к ```dh_auto_build``` следующее:
       ```
       dh_auto_build --parallel -- -j8
       ```
     Это передаст в make флаг ```-j8```

5. Можно запускать сборку:
    
    ```
    $ fakeroot debian/rules binary

    ```

   Результаты сборки появятся на уровень выше.
