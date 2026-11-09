> Исторический отчёт. Текущее состояние и следующий шаг — в [реестре](../../TASK_REGISTRY.md).

# Своя навигация собрана

Дата: 9 ноября 2026. Исполнение — удалённый ПК <remote-user>@astribot.

## Изоляция

- Compose project `al-isaac`, контейнер `al-isaac-navigation`.
- Официальный ROS Humble base image по digest.
- Свои зависимости Nav2/RTAB-Map и STag; образ команды не используется.
- STag C++ зафиксирован на commit `6214edc03095b6c488d3d95d866aa1a84228e574`.
- Образ навигации: `al-isaac-navigation:humble`, ID
  `sha256:ad84b7afe50b07ecddc07482b6ba75173badf885cabd08c2d8b55bc96e3ba827`.
- Пользователь контейнера: UID/GID 1005; исходники смонтированы read-only в /repo.
- Build/install/log — в `isaac/runtime/navigation`, не в исходном navigation/src.
- ROS_DOMAIN_ID=71, Fast DDS UDPv4.

## Проверки

Собраны все 11 ROS-пакетов из navigation/src и собственного lidar_stitching.
Отдельно проверены установленные al_nav2_plugins, active_localization, al_eval,
al_odom_noise и lidar_stitching. Алгоритмы и параметры навигации не менялись.

- C++ тесты al_nav2_plugins: 35, ошибок и пропусков нет.
- Python active_localization, al_odom_noise, al_eval и геометрия: 169 passed, 1 skipped.
- Дополнительный тест геометрии: поза всех joints совпадает с исходным Gazebo rsp.launch.py.
- Подготовленная модель: 38 links, 37 fixed joints, 18 уникальных mesh-файлов.
- Gazebo plugins в сгенерированном URDF удалены, исходный URDF не меняется.
- Полное дерево transforms сохранено в runtime manifest, включая torso_rgbd и оба лидара.

Сборка и модульные тесты не подтверждают работу Nav2 в реальном ROS-графе Isaac.
Интеграционная проверка будет после движения базы и сенсоров.

## Команды

```bash
cd ~/projects/Active-Localisation
bash isaac/build_navigation.sh
bash isaac/test_navigation.sh
docker compose -f isaac/compose.yaml exec navigation bash
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
```

Пакеты apt зафиксированы в `isaac/runtime/evidence/navigation-dpkg.txt` после сборки.
Версии apt не закреплены в Dockerfile; при полной пересборке позже они могут измениться.
Сборка STag-пакета выдаёт предупреждения о неиспользованных аргументах/переменной,
но компиляция и линковка завершились успешно.
