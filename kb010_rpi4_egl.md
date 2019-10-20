# EGL на Raspberry Pi 4

Как уже [упоминалось ранее](kb009_ros_opengl.md), EGL - система для получения контекста OpenGL (ES). Используется она преимущественно в "embedded" (если так можно назвать всякие телефоны и прочие raspberry, на которых крутится вполне себе "взрослый" Linux, часто с не менее "взрослым" GNU) и не лишена своих приколов. Далее будет описан опыт работы с EGL на Raspberry Pi, к которой не подключен монитор (то есть работает RPi в headless-режиме).

## Пара слов про headless

Вообще говоря, OpenGL (ES) хоть и не говорит ничего про взаимодействие с оконной системой, но предполагает (большую часть времени), что какая-то поверхность для рисования у него есть. И большую часть времени такая поверхность действительно есть - это окно в какой-то оконной системе (ну или на худой конец целый экран, хотя это крайне редко). Собственно, EGL отвечает за связь с оконной системой (или чем-то, что её заменяет), и когда этой оконной системы нет, начинаются всякие проблемы.

Разумеется, в headless-режиме у нас нет ни оконной системы, ни экрана, на который надо что-то выводить, и нам приходится надеяться на какие-то расширения EGL, которые помогут всё-таки создать контекст.

## Что было раньше (RPi 1, 2, 3, zero)

Более старые модели Raspberry Pi поставлялись с проприетарными драйверами от Broadcom. Эти драйверы были весьма ужасны (могли повиснуть при компиляции абсолютно валидного шейдера и отправить всю систему в kernel panic без особых на то причин), но headless у них включался вообще без проблем:

```c++
#include <EGL/egl.h>

void create_context()
{
    EGLdisplay dpy = eglGetDisplay(EGL_DEFAULT_DISPLAY); // возвращает валидный display
    /* далее - eglInitialize, eglChooseConfig, eglCreateContext, eglMakeCurrent */
}
```

Больше того, проприетарные драйвера [ожидают, что в eglGetDisplay передадут EGL_DEFAULT_DISPLAY](https://github.com/raspberrypi/userland/blob/2448644657e5fbfd82299416d218396ee1115ece/interface/khronos/common/linux/khrn_client_platform_linux.c#L467) (ссылка на реализацию `khrn_platform_set_display_id`, вызываемую в [реализации eglGetDisplay](https://github.com/raspberrypi/userland/blob/2448644657e5fbfd82299416d218396ee1115ece/interface/khronos/egl/egl_client_cr.c#L170)), [с другими значениями display они не дружат](https://github.com/raspberrypi/userland/blob/master/interface/khronos/egl/egl_client.c#L96). Соответственно, во всех примерах используется `eglGetDisplay(EGL_DEFAULT_DISPLAY)`, все рады и счастливы.

Вернее, почти все. Потому что в какой-то момент для Raspberry Pi появились [открытые драйвера mesa](https://gitlab.freedesktop.org/mesa/mesa/tree/master/src/gallium/drivers/vc4), которые гораздо лучше работали с X11 и даже давали какую-то реализацию OpenGL 2.1 (не ES!), но со старым кодом так просто не работали. Впрочем, все как-то не особо парились, ибо возможность скомпилировать софт десятилетней давности на RPi без изменений для всех перевешивала здравый смысл.

## Что произошло дальше

Летом 2019 года Raspberry Pi Foundation выпустила Raspberry Pi 4. Среди нововведений - новый GPU, для которого, внезапно, не было закрытого драйвера! Всем сказали: "Идите в X11 и используйте там Mesa".

Для большинства казуальных пользователей это было действительно здорово, ибо теперь софт десятилетней давности ещё и почти не тормозил. Правда, по умолчанию даже "швабодный" драйвер выключен, и рисуется всё силами хоть и более сильного (по сравнению с предыдущими моделями), но всё ещё недотягивающего до нормального видеочипа CPU.

К слову, примеры кода, использующие GPU, перестали работать на Raspberry Pi 4.

Тем не менее, жить дальше как-то надо, поэтому я начал смотреть, что можно сделать дальше.

### Старый код, новые фейлы

Вообще говоря, EGL [ничего не говорит](https://www.khronos.org/registry/EGL/sdk/docs/man/html/eglGetDisplay.xhtml) про гарантию поддержки `EGL_DEFAULT_DISPLAY`. Значительная часть "разумных" реализаций эту константу поддерживает, но с Raspberry Pi 4 + Mesa такой фокус не прокатывает - по крайней мере, без X11 и подключенного монитора.

Вызов `eglGetDisplay(EGL_DEFAULT_DISPLAY)` вернёт какой-то указатель, но `eglInitialize` выдаст ошибку при передаче этого дисплея. Fail.

(Кстати, из-за этого огромное количество кода, витающего в интернетах, не будет работать - включая [примеры с сайта Khronos](https://www.khronos.org/registry/EGL/sdk/docs/man/html/eglIntro.xhtml))

### Попытка №2 - gbm

Беглый поиск по ключевым словам "Raspberry Pi 4 OpenGL ES" приводит к [обсуждению на форуме Raspberry](https://www.raspberrypi.org/forums/viewtopic.php?t=243707) по поводу использования некого GBM как бэкенда для EGL/OpenGL ES. В качестве примера использования GBM ссылаются на [`kmscube`](https://gitlab.freedesktop.org/mesa/kmscube/).

Но что такое вообще этот GBM?

Статья в Википедии про mesa [упоминает термин generic buffer management](https://en.wikipedia.org/wiki/Mesa_(computer_graphics)#Generic_Buffer_Management). Вроде как это прослойка для DRM (или DRI?), который, в свою очередь, является интерфейсом, который предоставляет ядро linux.

В общем, понятно одно: в мире Linux прям очень любят трёхбуквенные сокращения. KMS, DRM, DRI, GBM. Все как-то связаны, но как именно - непонятно.

В сухом остатке получается следующее: `kmscube` на Raspberry Pi 4 с подключенным дисплеем (и без X11!) заработал. Без подключенного дисплея - фиг. Поскольку нам интересен именно headless-режим, двигаемся дальше.

### EGL_MESA_platform_surfaceless

Среди расширений, которые есть в Mesa, довольно интересным поначалу показалось [`EGL_MESA_platform_surfaceless`](https://www.khronos.org/registry/EGL/extensions/MESA/EGL_MESA_platform_surfaceless.txt) - судя по описанию, это расширение позволяет создавать контексты, для которых нет "поверхностей", на которых происходит рисование. Это нам и не страшно - всё равно мы не собираемся ничего выводить на экран, поскольку экрана этого у нас нет.

(здесь уместно сказать, что я успел к этому моменту собрать Mesa из исходников - вдруг в репозитории всё уже десять раз протухло? Впрочем, есть подозрение, что это было излишним, и всё, что будет написано далее, прекрасно работает на той версии драйверов, которая есть в Raspbian Buster)

Итак, для использования `EGL_MESA_platform_surfaceless` нам нужно использовать `eglGetPlatformDisplay` вместо `eglGetDisplay`. Не очень-то здорово, но ладно, пишем:

```c++
EGLDisplay dpy = eglGetPlatformDisplay(EGL_MESA_platform_surfaceless, EGL_DEFAULT_DISPLAY, NULL);
```

И - о чудо! У нас теперь есть display handle, работает `eglInitialize` и... ну, в общем-то, всё.

Вызов `eglChooseConfig` с любым attribList'ом говорит нам, что подходящих конфигураций дисплея - 0. То есть вроде как подключение к дисплею есть, а вроде как и без толку оно.

### Strace в помощь

Среди сообщений, которые выдавались мне при попытке добыть surfaceless-контекст, одно было весьма любопытным: `libEGL warning: no hardware driver found`. "Как так - no hardware driver, вот же он, `v3d` называется," - недоумевал я. "Ну ладно, может, `strace` что-то покажет?"

`strace` действительно что-то показал: похоже, что libEGL пытался открывать файлы `/dev/dri/card0`, `/dev/dri/card1` и `/dev/dri/renderD128`. Первые два отправляли libEGL читать что-то из `/sys`, последнее же устройство отказалось открываться под предлогом нехватки привелегий. Оказалось, что права на эти устройства следующие:

```bash
$ ls -lah /dev/dri/
total 0
drwxr-xr-x  3 root root        120 Oct 20 11:15 .
drwxr-xr-x 17 root root       3.9K Oct 20 11:15 ..
drwxr-xr-x  2 root root        100 Oct 20 11:15 by-path
crw-rw----  1 root video  226,   0 Oct 20 11:15 card0
crw-rw----  1 root video  226,   1 Oct 20 11:15 card1
crw-rw----  1 root render 226, 128 Oct 20 11:15 renderD128
```

Пользователь `pi` уже состоял в группе `video`, но для `renderD128` нужна группа `render`! Ну ладно, `usermod -aG render $USER`, `sudo reboot`, перезапуск программы - конфигов по-прежнему 0, но предупреждение о нехватке аппаратного драйвера ушло. Прогресс!

### Луч надежды из рандомного блога

Получается, что libEGL что-то для себя находил, когда пытался открыть этот `/dev/dri/renderD128`. А что будет, если попробовать руками это устройство открыть? Может, кто-то уже пробовал это сделать до меня?

Беглый поиск показал [чей-то блог](https://blogs.igalia.com/elima/) на igalia.com (название, которое бросилось в глаза после [статьи про драйверы в блоге Raspberry Pi](https://www.raspberrypi.org/blog/vc4-and-v3d-opengl-drivers-for-raspberry-pi-an-update/)). [Сама статья](https://blogs.igalia.com/elima/2016/10/06/example-run-an-opengl-es-compute-shader-on-a-drm-render-node/) была больше про запуск compute-шейдеров в headless-режиме на абстрактной карточке, которая, тем не менее, выдавала устройство `/dev/dri/renderD128`. Может, и на Raspberry Pi 4 это прокатит?

Дело за малым: пишем:

```c++
int32_t fd = open("/dev/dri/renderD128", O_RDWR);
struct gbm_device *gbm = gbm_create_device(fd);
EGLDisplay dpy = eglGetPlatformDisplay(EGL_PLATFORM_GBM_MESA, gbm, NULL);
EGLint major, minor;
eglInitialize(dpy, &major, &minor);
eglChooseConfig(display, attribList, config, MAX_NUM_CONFIG, &num_config);
/*...*/
```

В `attribList` кладём только `{EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT, EGL_NONE}`. Смотрим в num_config - аж 32 конфига нам подходят! Создаём контекст с первым попавшимся, смотрим в строки OpenGL ES, видим:

```txt
Vendor: Broadcom
Version: OpenGL ES 3.1 Mesa 19.3.0-devel (git-3f8f52b241)
Renderer: V3D 4.2
Shading language version: OpenGL ES GLSL ES 3.10
Extensions: GL_EXT_blend_minmax GL_EXT_multi_draw_arrays GL_EXT_texture_format_BGRA8888 GL_OES_compressed_ETC1_RGB8_texture GL_OES_depth24 GL_OES_element_index_uint GL_OES_fbo_render_mipmap GL_OES_mapbuffer GL_OES_rgb8_rgba8 GL_OES_standard_derivatives GL_OES_stencil8 GL_OES_texture_3D GL_OES_texture_float GL_OES_texture_half_float GL_OES_texture_half_float_linear GL_OES_texture_npot GL_OES_vertex_half_float GL_EXT_texture_sRGB_decode GL_OES_EGL_image GL_OES_depth_texture GL_OES_packed_depth_stencil GL_EXT_texture_type_2_10_10_10_REV GL_OES_get_program_binary GL_APPLE_texture_max_level GL_EXT_discard_framebuffer GL_EXT_read_format_bgra GL_EXT_frag_depth GL_NV_fbo_color_attachments GL_OES_EGL_image_external GL_OES_EGL_sync GL_OES_vertex_array_object GL_EXT_occlusion_query_boolean GL_EXT_texture_rg GL_EXT_unpack_subimage GL_NV_draw_buffers GL_NV_read_buffer GL_NV_read_depth GL_NV_read_depth_stencil GL_NV_read_stencil GL_EXT_draw_buffers GL_EXT_map_buffer_range GL_KHR_debug GL_KHR_texture_compression_astc_ldr GL_OES_depth_texture_cube_map GL_OES_required_internalformat GL_OES_surfaceless_context GL_EXT_color_buffer_float GL_EXT_sRGB_write_control GL_EXT_separate_shader_objects GL_EXT_shader_implicit_conversions GL_EXT_shader_integer_mix GL_EXT_base_instance GL_EXT_compressed_ETC1_RGB8_sub_texture GL_EXT_draw_elements_base_vertex GL_EXT_primitive_bounding_box GL_EXT_shader_io_blocks GL_EXT_texture_border_clamp GL_EXT_texture_norm16 GL_KHR_context_flush_control GL_NV_image_formats GL_OES_draw_elements_base_vertex GL_OES_primitive_bounding_box GL_OES_shader_io_blocks GL_OES_texture_border_clamp GL_OES_texture_stencil8 GL_OES_texture_storage_multisample_2d_array GL_EXT_buffer_storage GL_EXT_float_blend GL_KHR_no_error GL_KHR_texture_compression_astc_sliced_3d GL_OES_EGL_image_external_essl3 GL_OES_shader_image_atomic GL_MESA_shader_integer_functions GL_KHR_parallel_shader_compile GL_MESA_framebuffer_flip_y GL_EXT_texture_query_lod
```

Самое главное: `Renderer: V3D 4.2`! У нас есть аппаратное ускорение! Ещё и `ES 3.1`, можно вообще compute shader'ы заводить (сомневаюсь, что они заработают, но сам факт очень приятен).

(с более старым драйвером вывод немного иной, представлен ниже)

```txt
Vendor: Broadcom
Version: OpenGL ES 3.0 Mesa 19.2.0-rc1
Renderer: V3D 4.2
Shading language version: OpenGL ES GLSL ES 3.00
Extensions: GL_EXT_blend_minmax GL_EXT_multi_draw_arrays GL_EXT_texture_format_BGRA8888 GL_OES_compressed_ETC1_RGB8_texture GL_OES_depth24 GL_OES_element_index_uint GL_OES_fbo_render_mipmap GL_OES_mapbuffer GL_OES_rgb8_rgba8 GL_OES_standard_derivatives GL_OES_stencil8 GL_OES_texture_3D GL_OES_texture_float GL_OES_texture_half_float GL_OES_texture_half_float_linear GL_OES_texture_npot GL_OES_vertex_half_float GL_EXT_texture_sRGB_decode GL_OES_EGL_image GL_OES_depth_texture GL_OES_packed_depth_stencil GL_EXT_texture_type_2_10_10_10_REV GL_OES_get_program_binary GL_APPLE_texture_max_level GL_EXT_discard_framebuffer GL_EXT_read_format_bgra GL_EXT_frag_depth GL_NV_fbo_color_attachments GL_OES_EGL_image_external GL_OES_EGL_sync GL_OES_vertex_array_object GL_EXT_occlusion_query_boolean GL_EXT_texture_rg GL_EXT_unpack_subimage GL_NV_draw_buffers GL_NV_read_buffer GL_NV_read_depth GL_NV_read_depth_stencil GL_NV_read_stencil GL_EXT_draw_buffers GL_EXT_map_buffer_range GL_KHR_debug GL_KHR_texture_compression_astc_ldr GL_OES_depth_texture_cube_map GL_OES_required_internalformat GL_OES_surfaceless_context GL_EXT_color_buffer_float GL_EXT_sRGB_write_control GL_EXT_separate_shader_objects GL_EXT_shader_integer_mix GL_EXT_base_instance GL_EXT_compressed_ETC1_RGB8_sub_texture GL_EXT_draw_elements_base_vertex GL_EXT_texture_border_clamp GL_KHR_context_flush_control GL_OES_draw_elements_base_vertex GL_OES_texture_border_clamp GL_OES_texture_stencil8 GL_EXT_float_blend GL_KHR_no_error GL_KHR_texture_compression_astc_sliced_3d GL_OES_EGL_image_external_essl3 GL_MESA_shader_integer_functions GL_KHR_parallel_shader_compile GL_EXT_texture_query_lod
```

Пример кода, который позволил мне это всё получить, лежит [в моих гистах](https://gist.github.com/sfalexrog/b0957dc8f6fc0b04e6e724ce4a09e769). Пока что это просто создание EGL-контекста, никаких операций с OpenGL, но это уже прогресс.

Для успешной сборки нужно линковать библиотеки `libEGL`, `libGLESv2`, `libgbm`. `ldd` на бинарнике выдаёт:

```bash
$ ldd ./egl_test_gbm
    linux-vdso.so.1 (0xbef7f000)
    /usr/lib/arm-linux-gnueabihf/libarmmem-${PLATFORM}.so => /usr/lib/arm-linux-gnueabihf/libarmmem-v7l.so (0xb6f81000)
    libGLESv2.so.2 => /usr/local/lib/arm-linux-gnueabihf/libGLESv2.so.2 (0xb6f67000)
    libEGL.so.1 => /usr/local/lib/arm-linux-gnueabihf/libEGL.so.1 (0xb6f23000)
    libgbm.so.1 => /usr/local/lib/arm-linux-gnueabihf/libgbm.so.1 (0xb6f06000)
    libstdc++.so.6 => /usr/lib/arm-linux-gnueabihf/libstdc++.so.6 (0xb6d98000)
    libm.so.6 => /lib/arm-linux-gnueabihf/libm.so.6 (0xb6d16000)
    libgcc_s.so.1 => /lib/arm-linux-gnueabihf/libgcc_s.so.1 (0xb6ce9000)
    libc.so.6 => /lib/arm-linux-gnueabihf/libc.so.6 (0xb6b9b000)
    libglapi.so.0 => /usr/local/lib/arm-linux-gnueabihf/libglapi.so.0 (0xb6b5c000)
    libexpat.so.1 => /lib/arm-linux-gnueabihf/libexpat.so.1 (0xb6b1b000)
    libX11-xcb.so.1 => /usr/lib/arm-linux-gnueabihf/libX11-xcb.so.1 (0xb6b09000)
    libxcb.so.1 => /usr/lib/arm-linux-gnueabihf/libxcb.so.1 (0xb6ada000)
    libxcb-dri2.so.0 => /usr/lib/arm-linux-gnueabihf/libxcb-dri2.so.0 (0xb6ac6000)
    libxcb-xfixes.so.0 => /usr/lib/arm-linux-gnueabihf/libxcb-xfixes.so.0 (0xb6aaf000)
    libdrm.so.2 => /usr/lib/arm-linux-gnueabihf/libdrm.so.2 (0xb6a90000)
    libwayland-client.so.0 => /usr/lib/arm-linux-gnueabihf/libwayland-client.so.0 (0xb6a76000)
    libwayland-server.so.0 => /usr/lib/arm-linux-gnueabihf/libwayland-server.so.0 (0xb6a59000)
    libdl.so.2 => /lib/arm-linux-gnueabihf/libdl.so.2 (0xb6a46000)
    libxcb-dri3.so.0 => /usr/lib/arm-linux-gnueabihf/libxcb-dri3.so.0 (0xb6a32000)
    libxcb-present.so.0 => /usr/lib/arm-linux-gnueabihf/libxcb-present.so.0 (0xb6a1f000)
    libxcb-sync.so.1 => /usr/lib/arm-linux-gnueabihf/libxcb-sync.so.1 (0xb6a09000)
    libxshmfence.so.1 => /usr/lib/arm-linux-gnueabihf/libxshmfence.so.1 (0xb69f7000)
    libpthread.so.0 => /lib/arm-linux-gnueabihf/libpthread.so.0 (0xb69cd000)
    /lib/ld-linux-armhf.so.3 (0xb6f96000)
    libXau.so.6 => /usr/lib/arm-linux-gnueabihf/libXau.so.6 (0xb69ba000)
    libXdmcp.so.6 => /usr/lib/arm-linux-gnueabihf/libXdmcp.so.6 (0xb69a5000)
    libffi.so.6 => /usr/lib/arm-linux-gnueabihf/libffi.so.6 (0xb698d000)
    librt.so.1 => /lib/arm-linux-gnueabihf/librt.so.1 (0xb6976000)
    libbsd.so.0 => /usr/lib/arm-linux-gnueabihf/libbsd.so.0 (0xb694e000)
```

(со старыми библиотеками - ниже)

```bash
$ LD_LIBRARY_PATH=/usr/lib/arm-linux-gnueabihf ldd ./egl_test_gbm
    linux-vdso.so.1 (0xbefbe000)
    /usr/lib/arm-linux-gnueabihf/libarmmem-${PLATFORM}.so => /usr/lib/arm-linux-gnueabihf/libarmmem-v7l.so (0xb6ee2000)
    libGLESv2.so.2 => /usr/lib/arm-linux-gnueabihf/libGLESv2.so.2 (0xb6ec2000)
    libEGL.so.1 => /usr/lib/arm-linux-gnueabihf/libEGL.so.1 (0xb6ea3000)
    libgbm.so.1 => /usr/lib/arm-linux-gnueabihf/libgbm.so.1 (0xb6e87000)
    libstdc++.so.6 => /usr/lib/arm-linux-gnueabihf/libstdc++.so.6 (0xb6d40000)
    libm.so.6 => /lib/arm-linux-gnueabihf/libm.so.6 (0xb6c97000)
    libgcc_s.so.1 => /lib/arm-linux-gnueabihf/libgcc_s.so.1 (0xb6c6a000)
    libc.so.6 => /lib/arm-linux-gnueabihf/libc.so.6 (0xb6b1c000)
    libGLdispatch.so.0 => /usr/lib/arm-linux-gnueabihf/libGLdispatch.so.0 (0xb6a8b000)
    /lib/ld-linux-armhf.so.3 (0xb6ef7000)
    libdl.so.2 => /lib/arm-linux-gnueabihf/libdl.so.2 (0xb6a78000)
    libpthread.so.0 => /lib/arm-linux-gnueabihf/libpthread.so.0 (0xb6a4e000)
    libdrm.so.2 => /usr/lib/arm-linux-gnueabihf/libdrm.so.2 (0xb6a2f000)
    libwayland-server.so.0 => /usr/lib/arm-linux-gnueabihf/libwayland-server.so.0 (0xb6a12000)
    libexpat.so.1 => /lib/arm-linux-gnueabihf/libexpat.so.1 (0xb69d1000)
    libffi.so.6 => /usr/lib/arm-linux-gnueabihf/libffi.so.6 (0xb69b9000)
    librt.so.1 => /lib/arm-linux-gnueabihf/librt.so.1 (0xb69a2000)
```
