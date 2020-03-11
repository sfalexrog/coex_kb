# OpenOCD, ST-Link и Pixracer

Документация PX4 пусть и [содержит некоторые сведения](http://dev.px4.io/master/en/software_update/stm32_bootloader.html) о процессе прошивки [и отладки](http://dev.px4.io/master/en/debug/gdb_debugging.html) Pixhawk-совместимых плат, но в этой документации в лучшем случае предлагают использовать Black Magic Probe или Dronecode Probe, да и про загрузчик пишут что-то вроде "специалисты знают, что делать, а если вы не знаете, то вам и не надо". Впрочем, на практике можно использовать доступные инструменты вроде ST-Link (в том числе китайские клоны).

## Подготовка отладчика

Поскольку работать мы будем на Ubuntu или каком-то ещё Linux'е, утилиты от ST нам не особо доступны (наверное, это не до конца правда, но почему-то мне не предлагали тысячи ссылок на официальные программы под Linux). В этом нет ничего страшного, поскольку существует проект [OpenOCD (Open On-Chip Debugger)](http://openocd.org/). Его можно собирать из исходников или использовать то, что ставится из пакетов - стоит лишь помнить, что конфигурационные файлы от bleeding edge-версии могут не поддерживаться в чём-то более старом. Ещё стоит взять [файл правил udev](https://repo.or.cz/openocd.git/blob/HEAD:/contrib/60-openocd.rules) и закинуть его к себе в систему, дабы не иметь проблем с правами доступа к нашему ST-Link'у.

Можно проверить, что OpenOCD работает с нашим отладчиком, запустив следующее:

```bash
$ openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/stm32f4x.cfg
Open On-Chip Debugger 0.10.0
Licensed under GNU GPL v2
For bug reports, read
    http://openocd.org/doc/doxygen/bugs.html
Info : auto-selecting first available session transport "hla_swd". To override use 'transport select <transport>'.
Info : The selected transport took over low-level target control. The results might differ compared to plain JTAG/SWD
adapter speed: 2000 kHz
adapter_nsrst_delay: 100
none separate
Info : Unable to match requested speed 2000 kHz, using 1800 kHz
Info : Unable to match requested speed 2000 kHz, using 1800 kHz
Info : clock speed 1800 kHz
Info : STLINK v2 JTAG v29 API v2 SWIM v7 VID 0x0483 PID 0x3748
Info : using stlink api v2
Info : Target voltage: 3.160630
Error: init mode failed (unable to connect to the target)
in procedure 'init'
in procedure 'ocd_bouncer'
```

Из важного здесь - пути к интерфейсу (`/usr/share/openocd/scripts/interface/stlink-v2.cfg`) и к описанию цели (`/usr/share/openocd/scripts/target/stm32f4x.cfg`). Если OpenOCD собирался из исходников и ставился в нестандартное место - их надо будет поправить, понятное дело. В остальном - подключение не удалось, поскольку устройства нет (да, OpenOCD не очень разговорчив).

Ради интереса можно попробовать подключиться к Pixracer'у, предварительно распаяв отладочный разъём. Нам нужны только пины SWDIO, SWCLK, GND. Питание будем подавать через USB. Успешное подключение будет выглядеть так:

```bash
$ openocd -f /usr/share/openocd/scripts/interface/stlink-v2.cfg -f /usr/share/openocd/scripts/target/stm32f4x.cfg
Open On-Chip Debugger 0.10.0
Licensed under GNU GPL v2
For bug reports, read
	http://openocd.org/doc/doxygen/bugs.html
Info : auto-selecting first available session transport "hla_swd". To override use 'transport select <transport>'.
Info : The selected transport took over low-level target control. The results might differ compared to plain JTAG/SWD
adapter speed: 2000 kHz
adapter_nsrst_delay: 100
none separate
Info : Unable to match requested speed 2000 kHz, using 1800 kHz
Info : Unable to match requested speed 2000 kHz, using 1800 kHz
Info : clock speed 1800 kHz
Info : STLINK v2 JTAG v29 API v2 SWIM v7 VID 0x0483 PID 0x3748
Info : using stlink api v2
Info : Target voltage: 3.159055
Info : stm32f4x.cpu: hardware has 6 breakpoints, 4 watchpoints
```

> **Note** Убедитесь, что у вас везде хорошие контакты. Иногда имеет смысл снять корпус Pixracer'а, чтобы соединение в разъёме Debug было более надёжным.

## Работа с OpenOCD

Взаимодействие с OpenOCD происходит преимущественно через сокеты. OpenOCD предоставляет интерфейс наподобие gdbserver'а, что означает, что к нему можно подключиться с помощью gdb (вернее, arm-none-eabi-gdb). Стандартный порт, на котором принимаются подключения, - 3333.

Сам OpenOCD также имеет множество различных функций, и работать с ними можно, например, через telnet (в зависимости от версии, порт подключения будет 4444 или 6666).

### "Заливка" прошивки inav

Проект [inav](https://github.com/iNavFlight/inav) хорош, например, тем, что "из коробочки" поддерживает [Pixracer](https://github.com/iNavFlight/inav/blob/master/src/main/target/PIXRACER/README.md). Кроме того, у них же есть какая-никакая [инструкция к отладчику](https://github.com/iNavFlight/inav/blob/master/docs/development/Hardware%20Debugging.md) и несколько скриптов, которые упрощают запуск. По сути, эту инструкцию можно использовать как руководство к "прошиванию".

Единственная проблема - изначально flash "закрыт" для записи, но "открыть" его можно, подключившись к OpenOCD:

```bash
$ telnet localhost 4444
Trying 127.0.0.1...
Connected to localhost.
Escape character is '^]'.
Open On-Chip Debugger
> stm32f2x
stm32f2x
  stm32f2x lock bank_id
  stm32f2x mass_erase bank_id
  stm32f2x options_read bank_id
  stm32f2x options_write bank_id user_options [ boot_add0 boot_add1]
  stm32f2x unlock bank_id
stm32f2x : command requires more arguments
in procedure 'stm32f2x'
> reset halt
Unable to match requested speed 2000 kHz, using 1800 kHz
Unable to match requested speed 2000 kHz, using 1800 kHz
adapter speed: 1800 kHz
target halted due to debug-request, current mode: Thread 
xPSR: 0x01000000 pc: 0x08001f14 msp: 0x20020000
> stm32f2x unlock 0
stm32f2x unlocked.
INFO: a reset or power cycle is required for the new settings to take effect.
```

### Возвращение PX4

PX4, по факту, состоит из двух "частей":

* "загрузчик" - та часть прошивки, которая находится по адресу `0x0800 0000`. Ей передаётся управление при включении микроконтроллера;
* основная прошивка - в нормальных условиях записывается "загрузчиком". Должна находиться по адресу `0x0800 4000`, но точный адрес стоит посмотреть в загрузчике (или вообще не смотреть, загрузчик сам в состоянии определить, что куда писать).

["Загрузчик"](https://github.com/PX4/Bootloader) и [основная прошивка](https://github.com/PX4/Firmware) собираются отдельно, из разных репозиториев. Собрать "загрузчик" можно при наличии того же компилятора (`arm-none-eabi-gcc`), которым собирается PX4. В директории `Bootloader` достаточно выполнить `make`, после чего перейти в директорию с артефактами сборки и записать "загрузчик" в flash-память контроллера:

```bash
$ cd build/px4fmuv4_bl
$ openocd -f interface/stlink-v2.cfg -f target/stm32f4x.cfg -c "program ./px4fmuv4_bl.bin verify reset exit 0x08000000"
Open On-Chip Debugger 0.10.0
Licensed under GNU GPL v2
For bug reports, read
	http://openocd.org/doc/doxygen/bugs.html
Info : auto-selecting first available session transport "hla_swd". To override use 'transport select <transport>'.
Info : The selected transport took over low-level target control. The results might differ compared to plain JTAG/SWD
adapter speed: 2000 kHz
adapter_nsrst_delay: 100
none separate
Info : Unable to match requested speed 2000 kHz, using 1800 kHz
Info : Unable to match requested speed 2000 kHz, using 1800 kHz
Info : clock speed 1800 kHz
Info : STLINK v2 JTAG v29 API v2 SWIM v7 VID 0x0483 PID 0x3748
Info : using stlink api v2
Info : Target voltage: 3.161129
Info : stm32f4x.cpu: hardware has 6 breakpoints, 4 watchpoints
Info : Unable to match requested speed 2000 kHz, using 1800 kHz
Info : Unable to match requested speed 2000 kHz, using 1800 kHz
adapter speed: 1800 kHz
target halted due to debug-request, current mode: Thread 
xPSR: 0x01000000 pc: 0x080362e8 msp: 0x10010000
Info : Unable to match requested speed 8000 kHz, using 4000 kHz
Info : Unable to match requested speed 8000 kHz, using 4000 kHz
adapter speed: 4000 kHz
** Programming Started **
auto erase enabled
Info : device id = 0x20016419
Info : flash size = 2048kbytes
Info : Dual Bank 2048 kiB STM32F42x/43x/469/479 found
target halted due to breakpoint, current mode: Thread 
xPSR: 0x61000000 pc: 0x20000046 msp: 0x10010000
wrote 16384 bytes from file ./px4fmuv4_bl.bin in 0.576494s (27.754 KiB/s)
** Programming Finished **
** Verify Started **
target halted due to breakpoint, current mode: Thread 
xPSR: 0x61000000 pc: 0x2000002e msp: 0x10010000
verified 10704 bytes in 0.043246s (241.713 KiB/s)
** Verified OK **
** Resetting Target **
Info : Unable to match requested speed 2000 kHz, using 1800 kHz
Info : Unable to match requested speed 2000 kHz, using 1800 kHz
adapter speed: 1800 kHz
shutdown command invoked
```

После этого основная часть прошивки возвращается через QGroundControl.
