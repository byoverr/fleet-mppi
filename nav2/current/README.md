# Сравнение CostCritic на Nav2 main

Контрольная версия — коммит [bdb75f435ecb](https://github.com/ros-navigation/navigation2/commit/bdb75f435ecb79d92266060801ba6d95e23447db), вершина main, проверенная 22 ноября 2026 года перед прогоном. В обеих сборках включён OpenMP, параметр критика `num_threads` равен 1. Единственное изменение алгоритма — консервативная проверка полигонального контура по битовой карте; по умолчанию она выключена.

## Результаты

Медиана CPU-времени одного вызова `CostCritic` по пяти повторам, в миллисекундах.

| Сцена | Траекторий | Штатный Nav2, мс CPU | Фильтр выкл., мс CPU | Фильтр вкл., мс CPU |
|---|---:|---:|---:|---:|
| Коридор | 32 | 0,019 | 0,023 | 0,017 |
| Коридор | 125 | 0,145 | 0,166 | 0,055 |
| Коридор | 2000 | 2,475 | 2,816 | 0,789 |
| У стены | 32 | 0,223 | 0,235 | 0,034 |
| У стены | 125 | 1,037 | 1,068 | 0,118 |
| У стены | 2000 | 15,972 | 17,021 | 1,751 |

[Штатная сборка](upstream.json) · [Оба режима опытной сборки](paired.json) · [Побитовое сравнение и флаги столкновений](equivalence.txt)

Для пересчёта таблицы запустите в корне Fleet MPPI `uv run --no-project python tools/report_costcritic.py`. Чтобы обработать свой прогон, передайте путь к каталогу с двумя одноимёнными JSON-файлами. Для перестроения графика из опубликованных данных выполните `uv run --project mpp --frozen python tools/plot_costcritic.py`.

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

## Карты и траектории на рисунке

[Координаты](scenes.csv) выгружены из тех же функций стенда, которые строят карту и генерируют входные траектории для замера. В каждой сцене взяты 32 траектории по 100 шагов и добавлена начальная поза. Строки `cell` задают центры клеток препятствий со стороной 0,1 м, `footprint` — вершины контура с защитным запасом, `path` — координаты центра для траектории `id` на шаге `step`. Координаты указаны в метрах. Рисунок показывает участок карты вокруг робота.

Чтобы заново построить рисунок из сохранённых координат, выполните из корня Fleet MPPI:

```bash
uv run --project mpp --frozen python tools/plot_scenes.py
```

Для новой выгрузки примените дополнительный патч после сборки измерительного стенда. Команды выполняются в том же ROS-пространстве и используют переменные из раздела «Сборка».

```bash
git -C src/navigation2 apply "$LAB/nav2/current/export-scenes.patch"
cmake --build build/nav2_mppi_controller --target fleet_cost_critic -j2
"$bench/fleet_cost_critic" --export-scenes scenes.csv
uv run --project "$LAB/mpp" --frozen python "$LAB/tools/plot_scenes.py" --input scenes.csv --output scenes.png
```
