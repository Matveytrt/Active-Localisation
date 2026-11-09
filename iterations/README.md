# Итерации переноса на Isaac Sim

Актуальная цель, решения и следующий шаг — только в [реестре](../TASK_REGISTRY.md).

| № | Шаг | Отчёт | Ревью |
|---|---|---|---|
| 000 | Аудит и план | [report](000-audit/report.md) | [review](000-audit/review.md) |
| 001 | Доступ и runtime | [report](001-runtime/report.md) | [review](001-runtime/review.md) |
| 002 | Своя навигация | [report](002-navigation/report.md) | [review](002-navigation/review.md) |
| 003 | Astribot и GPU-рендер | [report](003-robot-render/report.md) | [review](003-robot-render/review.md) |
| 004 | Реестр и ветка задачи | [report](004-task-registry/report.md) | [review](004-task-registry/review.md) |
| 005 | Движение, камера и два лидара | [report](005-base-sensors/report.md) | [review](005-base-sensors/review.md) |
| 006 | Собственный мир и реальные STag; ограниченная фронтальная метрология | [report](006-world-stag/report.md) | [review](006-world-stag/review.md) |
| 007 | Своя карта RTAB-Map и baseline pilot | [report](007-map-baseline/report.md) | [review](007-map-baseline/review.md) |
| 008 | Общий runner baseline/active/control | [report](008-runner-modes/report.md) | [review](008-runner-modes/review.md) |
| 009 | Воспроизведение основного протокола и отчёт о переносе | [report](009-reproduction/report.md) | [review](009-reproduction/review.md) |
| 010 | Короткое видео нового active demo | [report](010-video/report.md) | [review](010-video/review.md) |
| 011 | Видео с native active costmap и live stats | [report](011-video-costmap/report.md) | [review](011-video-costmap/review.md) |

Для нового шага создавать NNN-name/report.md и review.md.
Отчёт: задача, исходное состояние, изменения, команды/артефакты, фактические проверки,
результат и оставшаяся работа. Ревью: кто проверил, что сверено, замечания,
ограничения доказательств и решение о принятии. Новые факты добавлять с датой,
исторические результаты не переписывать под последующие успехи.
