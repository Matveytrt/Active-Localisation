# STag Data Analyzer

Пакет для анализа и визуализации данных экспериментов с детекцией STag маркеров.

## Возможности

- Парсинг JSONL логов детекции маркеров
- Подсчет статистики: FPS, количество детекций, одновременная видимость
- Генерация heatmap с использованием Plotly
- Усреднение данных по нескольким попыткам эксперимента
- Гибкая настройка через YAML конфигурацию

## Установка

```bash
cd /path/to/workspace
colcon build --packages-select stag_data_analyzer
source install/setup.bash
```

## Использование

```bash
# Запуск анализа с конфигурацией по умолчанию
ros2 run stag_data_analyzer analyze_data

# Запуск с кастомной конфигурацией
ros2 run stag_data_analyzer analyze_data --config /path/to/config.yaml
```

## Структура проекта

```
stag_data_analyzer/
├── core/               # Ядро: ConfigManager (загрузка YAML-конфигурации)
├── models/             # Модели данных (DatasetRoot и вложенные группы)
├── parsers/            # Парсеры JSONL (LogParser) и структуры папок (PathParser строит индекс данных DatasetRoot)
├── analyzers/          # Анализаторы статистики
└── visualizers/        # Генераторы визуализаций
```

## Конфигурация

Смотрите `config/analysis_config.yaml` для примера настройки параметров анализа.
