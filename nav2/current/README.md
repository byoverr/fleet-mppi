# Сравнение CostCritic на Nav2 main

Контрольная версия — коммит [bdb75f435ecb](https://github.com/ros-navigation/navigation2/commit/bdb75f435ecb79d92266060801ba6d95e23447db), вершина main, проверенная 22 ноября 2026 года перед прогоном. В обеих сборках включён OpenMP, параметр критика `num_threads` равен 1. Единственное изменение алгоритма — консервативная проверка полигонального контура по битовой карте; по умолчанию она выключена.

## Сборка

Нужна среда ROS 2 Rolling с colcon, зависимостями Nav2 и Google Benchmark. Ниже `LAB` — абсолютный путь к Fleet MPPI. Выполняйте команды в новом пустом ROS-пространстве. Измерительный патч добавляет две программы и опцию сборки. Контроллер в первом прогоне собирается из оригинальных исходников.

```bash
source /opt/ros/rolling/setup.bash
export MAKEFLAGS=-j2 CMAKE_BUILD_PARALLEL_LEVEL=2
LAB=/absolute/path/to/fleet-mppi
mkdir -p src
git clone https://github.com/ros-navigation/navigation2 src/navigation2
git -C src/navigation2 checkout --detach "$(cat "$LAB/nav2/current/upstream.sha")"
git -C src/navigation2 apply "$LAB/nav2/current/benchmarks.patch"
colcon build --executor sequential --packages-up-to nav2_mppi_controller --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DENABLE_OPENMP=ON -DFLEET_MPPI_BENCHMARKS=ON
source install/setup.bash
bench="$PWD/build/nav2_mppi_controller/fleet_benchmark"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp OMP_NUM_THREADS=1
flags=(--benchmark_min_time=0.20s --benchmark_repetitions=5 --benchmark_enable_random_interleaving=true --benchmark_out_format=json)
"$bench/fleet_cost_critic" --benchmark_filter='mask:0/' "${flags[@]}" --benchmark_out=upstream.json

git -C src/navigation2 apply "$LAB/nav2/current/footprint.patch"
colcon build --packages-select nav2_mppi_controller --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF -DENABLE_OPENMP=ON -DFLEET_MPPI_BENCHMARKS=ON
source install/setup.bash
"$bench/fleet_cost_equivalence"
"$bench/fleet_cost_critic" "${flags[@]}" --benchmark_out=paired.json
```

## Среда проведённого опыта

Apple M4, контейнер Docker с Ubuntu 26.04.1 arm64, ROS 2 Rolling, GCC 15.2.0. Контроллер и его зависимости из Nav2 собраны в режиме Release; Google Benchmark 1.9.1 сообщает debug для самой измерительной библиотеки. Оба бинарных файла используют одну и ту же библиотеку. Метрики Google Benchmark относятся к циклу измерения, включающему обнуление стоимостей и вызов критика. Это важно при сравнении малых выборок.

## Условия опыта

- Площадка 30 × 30 м, разрешение карты 0,1 м, полигональный корпус 3,66 × 1,30 м до прибавления запаса 0,1 м. Две сцены — коридор и движение вдоль стены. Для каждой сцены генератор создаёт одинаковые траектории при сравнении вариантов.
- За вызов критик оценивает 32, 125 или 2000 траекторий по 100 шагов. Проверяется каждая вторая поза. Карта постоянна в пределах измерения, битовая плоскость под текущую карту строится при каждом вызове с включённым фильтром. Фиксированные маски контура прогреваются до измерения.
- `upstream.json` содержит результаты штатного контроллера. В `paired.json` один и тот же бинарный файл запускается с фильтром и без него в перемешанном порядке. Основная метрика — суммарное CPU-время процесса на вызов. В журналах также приведены длительность вызова и p99.
- Проверка эквивалентности использует отдельные случайные карты и траектории. Сравниваются стоимость каждой траектории побитово, флаг столкновения каждой траектории и общий флаг отказа. Числа относятся к испытанным конфигурациям.
