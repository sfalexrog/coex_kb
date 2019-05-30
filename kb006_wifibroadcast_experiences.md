# Опыт работы с различными WifiBroadcast-системами

## Background

**Wifibroadcast** - общее название для систем, использующих адаптеры Wi-Fi в качестве радиоприёмников/радиопередатчиков (as opposed to the usual way of using them, наподобие создания сетей с какими-то правилами и гарантиями). Цель - организовать *ненадёжный*, возможно однонаправленный канал для передачи данных (сами данные могут прокидываться как, например, UDP-пакеты).

Для работы с системами Wifibroadcast требуется использовать адаптеры, поддерживающие [Monitor mode](https://en.wikipedia.org/wiki/Monitor_mode) и Packet injection. При этом поддержка должна быть и со стороны драйверов (чего очень часто не бывает, поскольку производители, которые драйверы пишут, не очень-то хотят, чтобы у пользователей был низкоуровневый доступ к железу).

## Существующие проекты

### Реализации WFB

1. [Блог-пост от Befinitiv](https://befinitiv.wordpress.com/wifibroadcast-analog-like-transmission-of-live-video-data/), показывающий работу Wifibroadcast-системы. Наверное, один из более ранних экспериментов на эту тему. [Репозиторий со сборщиком](https://bitbucket.org/befi/rpi_wifibroadcast_image_builder/src/default/) находится на битбакете и вообще использует Mercurial.
1. [Wifibroadcast от RespawnDespair](https://github.com/RespawnDespair/wifibroadcast-base) - одна из более распространённых реализаций Wifibroadcast. Собирается своим [image builder'ом](https://github.com/RespawnDespair/wifibroadcast-image-builder).
1. [Wifibroadcast от svpcom](https://github.com/svpcom/wifibroadcast) - отличающаяся от остальных реализация. Отличается, наверное, наличием шифрования и коррекции ошибок, удобством сборки. А, ну и есть режимы Video/Mavlink/IPoverWFB.

### Готовые дистрибутивы

1. [EZ-Wifibroadcast от rodzio1](https://github.com/rodizio1/EZ-WifiBroadcast) - видимо, тоже какое-то подобие сборщика/сборки. Есть подозрение, что имеет много общего с предыдущим пунктом.
1. [Open.HD от HD-Fpv](https://github.com/HD-Fpv/Open.HD) - использует вроде бы всё тот же Wifibroadcast, но теперь с поддержкой MAVLink 2.0. Почему-то на этом варианте наблюдалась меньшая дальность, если верить тем, кто с этим работал.

### Разное

1. [Nexmon](https://github.com/seemoo-lab/nexmon) - набор утилит и патчей для разблокировки Monitor mode в Broadcom'овских чипах. С ходу на Raspbian'е не получилось запустить, но всё можно исправить правильным ядром (см. ниже).

## Особенности/приколы

Для работы в режиме Wifibroadcast надо использовать патченные драйверы. Как правило, патчи есть в каждом сборщике дистрибутива, и заключаются они в следующем:

- отключается контроль мощности по регионам;
- добавляется мониторный режим (если его почему-то не было).

В основном поддерживаются адаптеры на базе RTL8812au, RT28xx. Могут поддерживаться atheros'ы и некоторые ralink'и.

В EZ-Wifibroadcast Builder'е происходит пересборка всего ядра, wifibroadcast от svpcom рекомендует пропатченный модуль ядра. Скорее всего, разумно собирать всё ядро сразу, так оно и нужной версии будет.