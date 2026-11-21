# Стенд Navigation2

Оба варианта основаны на коммите `2d3d1cc569b411b2b5f477be620c35796edb75c6` из [ros-navigation/navigation2](https://github.com/ros-navigation/navigation2). Патч и дополнительные исходники зафиксированы на этапе испытаний оптимизаций габарита, генерации и PathAlign (снимок экспериментальной серии 1c4f716). Сохранённые численные результаты получены на двух установленных в nav2lab сборках от той же базы. В более поздней лабораторной сборке также присутствовали отключённые по умолчанию опции выборки шума.

## Подготовка двух рабочих пространств

В среде ROS 2 Rolling с colcon, rosdep, Eigen и libbenchmark-dev задайте LAB равным абсолютному пути к каталогу `studies/drive-identification` внутри Fleet MPPI. Команды ниже предусматривают пустые ROS-пространства /ws и /ws_main внутри контейнера.

```bash
for ws in /ws_main /ws; do
  mkdir -p "$ws/src"
  git clone https://github.com/ros-navigation/navigation2 "$ws/src/navigation2"
  git -C "$ws/src/navigation2" checkout --detach 2d3d1cc569b411b2b5f477be620c35796edb75c6
done
pkg=/ws_main/src/navigation2/nav2_mppi_controller
cp "$LAB/nav2/baseline/package-CMakeLists.txt" "$pkg/CMakeLists.txt"
cp "$LAB/nav2/baseline/CMakeLists.txt" "$LAB/nav2/baseline/"*.cpp "$pkg/benchmark/"
git -C /ws/src/navigation2 apply --check "$LAB/nav2/controller.patch"
git -C /ws/src/navigation2 apply "$LAB/nav2/controller.patch"
cp -R "$LAB/nav2/nav2_mppi_controller/." /ws/src/navigation2/nav2_mppi_controller/
source /opt/ros/rolling/setup.bash
for ws in /ws_main /ws; do
  (cd "$ws" && rosdep install --from-paths src --ignore-src -r -y &&
    colcon build --packages-up-to nav2_mppi_controller --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON)
done
bash "$LAB/nav2/nav2-benchmark.sh"
```

Скрипт задаёт габарит AMR 0,7 × 0,5 м, чередует порядок сборок и выключает экспериментальное предварительное чтение карты (`NO_CCR=1`). Для длительных испытаний можно увеличить аргумент времени. Рассчитанное число роботов при 20 Гц описывает только вычисление команд.

**Статус проверки.** Указанный патч проверен командой git apply на чистой закреплённой базе. Эталонные измерения сняты в подготовленном контейнере ROS Rolling/Ubuntu 26.04 arm64. Совместимость с другими ветками и пересборка в сегодняшнем Rolling из пустого образа требуют отдельной проверки. Сборки содержат специальную настройку набора тестовых и измерительных целей и предназначены для эксперимента.
