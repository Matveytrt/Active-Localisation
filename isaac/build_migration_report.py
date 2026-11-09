"""Build the separate reproduction report from verified experiment evidence.

Use bundled reportlab/Pillow on the laptop, after copying the remote summary.
The existing Gazebo report is an input and is never modified.
"""
import argparse
import json
from pathlib import Path
import shutil
from xml.sax.saxutils import escape
from reportlab.lib import colors
from reportlab.lib.enums import TA_CENTER, TA_JUSTIFY
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.units import mm
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.platypus import (SimpleDocTemplate, Paragraph, Spacer, PageBreak,
                               Table, TableStyle, Image, KeepTogether)

REPO=Path(__file__).resolve().parents[1]
RUNTIME=REPO/'isaac/runtime'


PAGES = [
('Цель и границы воспроизведения', [
('p', 'Цель работы — воспроизвести решение Active Localisation для Astribot S1 в Isaac Sim, сохранив смысл эксперимента, ROS-интерфейсы и алгоритмы исходной реализации Gazebo. Перенос охватывает робота, движение, камеру, два лидара, мир с маркерами, новую карту RTAB-Map и три режима навигации. Исходный отчёт о Gazebo сохраняется как описание метода; этот документ посвящён процессу его воспроизведения.'),
('p', 'Платформа воспроизведения: Isaac Sim 5.1.0, ROS 2 Humble, Nav2 1.1.20 и RTAB-Map 0.23.13. Смена симулятора отделена от смены ROS-дистрибутива: алгоритмические узлы работают с прежними сообщениями и сервисами, а специфическая для Isaac часть реализована в адаптере.'),
('p', 'Основной принцип переноса — сначала восстановить измерения и управление, затем подключать прежние алгоритмы. Совпадение внешнего вида модели само по себе не подтверждает эквивалентность стенда: ошибка в системе координат датчика или в геометрическом размере маркера меняет эксперимент даже при правдоподобном изображении.'),
('h', 'Что сохранено'),
('p', 'Сохранены ROS 2 Humble, omnidirectional-команды vx, vy, wz, исходная модель дрейфа одометрии, реальный детектор STag, RTAB-Map с лидаром и HD17-ориентирами, Theta*, MPPI Omni, дерево поведения и восстановления Nav2. Плагины LocalizabilityLayer и LocalizationAwarePlanner перенесены без изменения их алгоритмов. Направление камеры по-прежнему меняется поворотом базы независимо от направления движения.'),
('contract', None),
('h', 'Что означает успешное воспроизведение'),
('p', 'Робот выполняет исходные сценарии через штатные действия NavigateToPose, а не через заранее записанную траекторию. Ориентиры обнаруживаются в изображении камеры. Оценка позы строится по шумной одометрии, лидару и принятым детекциям. Истинная поза служит входом модели шума и используется для проверки, но напрямую не передаётся в SLAM и Nav2. Сравнение baseline, active и control выполняется на одной новой карте с одинаковыми условиями начала эпизода.'),
('p', 'Колёсная контактная динамика, балансировка и манипуляции не входили в исходный навигационный эксперимент. В Isaac сохранена плоская кинематическая база и фиксированная конфигурация тела. Поэтому результат относится к воспроизведению навигационного задания, а не ко всем физическим возможностям Astribot.')]),
('Модель робота и движение базы', [
('p', 'За основу взяты URDF, исходная конфигурация позы и собственные mesh-файлы проекта. Модель импортирована в USD с сохранением структуры преобразований: 38 links, 37 фиксированных joints и 18 mesh-файлов. Поза рук и корпуса восстановлена из исходной конфигурации, чтобы не менять положение сенсоров относительно базы.'),
('robot', None),
('p', 'После импорта проверялись не только геометрия и масштаб, но и преобразования от base_link до камеры и лидаров. Высота base_link над полом сохранена равной 0,16 м. Перемещение в плоскости реализовано по тем же трём степеням свободы, которые использовались в Gazebo; боковая скорость не заменена ограничением дифференциальной базы.'),
('h', 'Движение и столкновения'),
('p', 'Навигация публикует обычные команды скорости. Адаптер интегрирует позу базы, ограничивает допустимые скорости и останавливает движение при потере команд. Для неподвижного мира применяется проверка прямоугольного footprint по ориентированным препятствиям с проверкой промежуточного движения. Эта проверка препятствует проходу базы через стену или стеллаж, но не моделирует силы контакта колёс.'),
('p', 'Общий профиль скорости составляет 0,6 м/с по x и y и 0,8 рад/с по yaw; одинаковые ограничения заданы базе, MPPI и сглаживателю. Это изменение исполнения относительно исходных значений явно разделено с алгоритмическими настройками и применяется во всех трёх режимах.'),
('h', 'Проверка движения'),
('p', 'Внешний ROS-тест проверил движение вперёд и вбок, вращение, остановку по watchdog, остановку перед препятствием и сброс эпизода. Отдельно проверено, что шумная одометрия и истинная поза различаются, а единственным владельцем TF odom → base_link остаётся исходный узел модели дрейфа.')]),
('Камера лидары и ориентиры', [
('p', 'Камера на торсе воспроизведена как RTX RGB/depth-камера с разрешением 640 × 480, горизонтальным полем зрения 1,089 рад и fx около 528,43 пикселя. Проверены optical-frame, CameraInfo и согласованность глубины. Сохранение этих параметров важно для исходного score видимости: он использует геометрическую площадь проекции маркера.'),
('h', 'Основная трудность с лидарами'),
('p', 'У обоих лидаров восстановлены исходные extrinsics, азимуты, углы по высоте, дальности и шум. Использован штатный PhysX Generic Sensor с пакетами из 1920 лучей на датчик, а затем прежний узел сшивки облаков. В Gazebo elevation положителен вверх, а в выбранном API PhysX положительный zenith направлен вниз. Прямое копирование углов давало неверную геометрию; при переносе elevation меняет знак.'),
('sensor', None),
('h', 'Размер и ориентация STag'),
('p', 'В мир перенесены исходные текстуры HD15 и HD17. Физический размер чёрного квадрата занимает 80% панели: для HD17 он равен 0,08 м, для HD15 — 0,064 м. Старое значение HD15 0,06 м не совпадало с геометрией текстуры и давало систематическую ошибку PnP; размер исправлен в адаптере, сам детектор сохранён.'),
('p', 'В контрольных фронтальных наблюдениях медианная ошибка переноса составила около 3–8 мм; отдельно проверен ненулевой HD17 ID1 с ошибкой 9,27 мм. Это локальные контрольные измерения. При боковых ракурсах сохранилась неоднозначность IPPE и скачки нормали, а сырые детекции иногда содержали ложные ID. В RTAB-Map передаются только исходно разрешённые HD17 IDs 1–5 с проверками свежести и выбросов. Идеальные позы маркеров из симулятора не подставляются вместо детекций.'),
('detection',None)]),
('Собственный мир и новая карта', [
('p', 'Мир восстановлен из исходного SDF, а не заменён похожим складом. Перенесены шесть стеллажей, четыре стены и 24 панели STag. Для DAE-геометрии проверены единицы, масштаб 0,001 и вложенные model/link/geometry-преобразования. Ориентированные collision boxes совпадают с исходными; визуальные mesh и поверхности препятствий проверялись отдельно.'),
('world', None),
('p', 'Для перенесённой сцены построена новая сенсорная карта RTAB-Map: 947 узлов и пять оптимизированных HD17-ориентиров. Использование старой Gazebo DB означало бы перенос результатов картирования, а не воспроизведение этапа в другом симуляторе. Положения ориентиров для планировщика экспортированы из оптимизированного графа новой карты.'),
('h', 'Память и полный граф'),
('p', 'Неограниченное накопление рабочей памяти и кэшей привело к прерыванию первого картирования. Введён общий профиль: MemoryThr = 200, map_filter_radius = 0,15 м, map_filter_angle = 10° и отключённая предварительная загрузка кэша карты. Сырые измерения продолжают записываться в DB. Этот профиль одинаков для baseline, active и control.'),
('p', 'После ограничения working memory обычный кэш оптимизированных поз содержал только часть ориентиров. Полный граф с long-term memory извлечён штатным API RTAB-Map и оптимизирован в отдельной копии DB. Проверки хешей подтвердили неизменность сырых данных, связей и исходных поз узлов. Ground truth и координаты объектов мира не использовались для оптимизации. Медианная ошибка XY пяти ориентиров относительно контрольных поверхностей сцены составила 1,74 см.')]),
('Воспроизведение протокола эксперимента', [
('p', 'Пилот shelves_south_wall подтвердил перенос до основной серии: все шесть целей достигнуты. Active изменил маршрут относительно control в среднем на 0,444 м, а yaw — на 0,952 рад на общем участке. Работа плагинов проверена по фактическому движению и журналам загрузки.'),
('p', 'Основная серия повторяет структуру Gazebo-задания: shelves_loop_a, три цели, шум ×4/×6, seeds 42–46 и три режима. Дисперсионные коэффициенты масштабируются как k², систематические ошибки — как k. Порядок режимов ротируется. Каждый эпизод получает новый симулятор, ROS-стек и отдельную копию карты; профили проверяются в живом ROS-графе.'),
('p', 'Начальное перемещение Gazebo на 0,6 м заменено один раз заданной известной позой сценария. Во время целей ручные команды и коррекции по истинной позе не подаются. Одинаковый seed не даёт одинакового дрейфа на разных путях; RNG MPPI этим адаптером не управляется.'),
('p', 'APE/RPE рассчитаны прежним pipeline на evo 1.38.0: XY, без выравнивания, интерполяция GT в моменты оценки с допуском 0,02 с, RPE через 1 м. Окно — от первой до последней цели, без начальной локализации. Аналитические контроли проверили сопоставление времени, отсутствие скрытого выравнивания и исключение разницы высоты.'),
('results', None),
('exceptions',None),
('p', 'Таблица служит проверкой работоспособности перенесённого задания. Изменение симулятора, карты, сенсорной реализации и профиля исполнения не позволяет приписать различия с численными результатами Gazebo одной причине. Исходный отчёт по Gazebo не заменяется новой таблицей.')]),
('Основные выводы о процессе переноса', [
('conclusion', None),
('h', 'Что помогло'),
('p', 'Перенос по контрактам ROS позволил сохранять алгоритмы и проверять каждый новый компонент отдельно. Исходные URDF/SDF, текстуры, конфигурация позы и параметры камеры дали проверяемую геометрическую основу. Внешние ROS-тесты выявляли ошибки, которые не видны по скриншоту: знак углов лидара, направление optical-frame, свежесть TF, владельцев преобразований и реальную частоту сообщений.'),
('p', 'Разделение ground truth и оценки, новая карта из измерений, неизменная исходная DB и парные запуски защитили эксперимент от подмены локализации идеальной позой. Раздельные отчёты этапов и сохранение неудачных запусков позволили отличить предварительную диагностику от принятого результата.'),
('h', 'Наиболее существенные технические трудности'),
('p', 'Первой была геометрическая эквивалентность: разные соглашения углов, вложенные преобразования mesh, размер маркера и ориентация сенсоров. Второй — временной и информационный контракт: /clock, timestamps, QoS CameraInfo, TF и реальные детекции. Третьей — воспроизводимый жизненный цикл: готовность рендера наступает позже старта приложения, обнаружение action server раньше активации Nav2, а остановка ROS-обёртки не всегда завершает дочерний процесс.'),
('p', 'Производительность также влияла на достоверность стенда. Основным источником низкого real-time factor оказалась валидация миллионов байтов Python при присвоении ROS Image.data, а не рендер. Передача через штатный array.array(\'B\') подняла RTF в диагностике примерно с 0,07 до 0,99 и восстановила частоты камеры и лидаров. Готовность каждого эпизода теперь определяется валидными сенсорными данными и состояниями lifecycle, а не фиксированной паузой.'),
('h', 'Границы результата'),
('p', 'Подтверждено воспроизведение навигационного эксперимента на одной статичной сцене. Не подтверждены полная контактная динамика Astribot, качество PnP во всех ракурсах, длительная эксплуатационная надёжность и универсальное преимущество active localization. Сравнение эффективности метода требует учитывать интервалы парных различий и ограничения числа seeds; большее число детекций само по себе не доказывает снижение ошибки позы.'),
('h', 'Материалы воспроизведения'),
('p', 'Исходный отчёт Active Localization в Gazebo и Nav2 от 20.10.2026; репозиторий Matveytrt/Active-Localisation, ветка isaac; TASK_REGISTRY.md и отчёты iterations/005–009. Исходные Gazebo algorithms сохранены. Версии, конфигурации, контрольные суммы, результаты действий и сырые траектории каждого эпизода сохранены отдельно от исходников.'),
('commands', None)])]


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--summary',required=True)
    parser.add_argument('--output',default=str(REPO/'output/pdf/IsaacReproduction.pdf'))
    parser.add_argument('--draft',action='store_true')
    args=parser.parse_args()
    summary=json.loads(Path(args.summary).read_text())
    assert args.draft or (summary['status']=='complete' and summary['evaluated_jobs']==30 and summary['audit_failures']==0)
    if not args.draft:
        assert all(summary['groups'][scale][mode]['runs']==5 for scale in ('4','6') for mode in ('baseline','active','control'))
    out=Path(args.output);out.parent.mkdir(parents=True,exist_ok=True)
    fonts=Path('C:/Windows/Fonts')
    for name,file in [('Body','times.ttf'),('Bold','timesbd.ttf'),('Italic','timesi.ttf')]:
        pdfmetrics.registerFont(TTFont(name,str(fonts/file)))
    pdfmetrics.registerFontFamily('Body',normal='Body',bold='Bold',italic='Italic',boldItalic='Bold')
    styles={
        'p':ParagraphStyle('Body',fontName='Body',fontSize=10.4,leading=13.3,alignment=TA_JUSTIFY,spaceAfter=7),
        'h':ParagraphStyle('Subheading',fontName='Bold',fontSize=11.2,leading=14,spaceBefore=8,spaceAfter=6,keepWithNext=True),
        'title':ParagraphStyle('Title',fontName='Bold',fontSize=18,leading=22,alignment=TA_CENTER,spaceAfter=8),
        'section':ParagraphStyle('Section',fontName='Bold',fontSize=13.3,leading=17,spaceAfter=10),
        'caption':ParagraphStyle('Caption',fontName='Italic',fontSize=8.7,leading=11,spaceBefore=4,spaceAfter=8),
        'cell':ParagraphStyle('Cell',fontName='Body',fontSize=8.5,leading=10.5),
    }
    story=[];markdown=['# Воспроизведение Active Localisation в Isaac Sim','', '9 ноября 2026','']
    width=A4[0]-36*mm

    def plain(text):
        value=str(text)
        for dash in ('\u2010','\u2011','\u2012','\u2013','\u2014'):
            value=value.replace(dash,'-')
        return value

    def para(text,kind='p'):
        text=plain(text)
        story.append(Paragraph(escape(text).replace('\n','<br/>'),styles[kind]));markdown.extend([('### ' if kind=='h' else '')+text,''])

    def table(data,widths=None):
        data=[[plain(v) for v in row] for row in data]
        rows=[[Paragraph(escape(str(v)),styles['cell']) for v in row] for row in data]
        t=Table(rows,colWidths=widths or [width/len(data[0])]*len(data[0]),hAlign='LEFT')
        t.setStyle(TableStyle([('GRID',(0,0),(-1,-1),.35,colors.HexColor('#cccccc')),
            ('BACKGROUND',(0,0),(-1,0),colors.HexColor('#e8edf2')),
            ('VALIGN',(0,0),(-1,-1),'MIDDLE'),('LEFTPADDING',(0,0),(-1,-1),6),
            ('RIGHTPADDING',(0,0),(-1,-1),6),('TOPPADDING',(0,0),(-1,-1),5),('BOTTOMPADDING',(0,0),(-1,-1),5)]))
        story.append(t);story.append(Spacer(1,7))
        markdown.append('| '+' | '.join(str(v) for v in data[0])+' |')
        markdown.append('|'+'---|'*len(data[0]))
        markdown.extend('| '+' | '.join(str(v) for v in row)+' |' for row in data[1:]);markdown.append('')

    def picture(path,caption,w=width,h=None):
        caption=plain(caption)
        from PIL import Image as PILImage
        iw,ih=PILImage.open(path).size
        h=h or w*ih/iw
        story.append(KeepTogether([Image(str(path),width=w,height=h),Paragraph(escape(caption),styles['caption'])]))
        if not args.draft:
            destination=REPO/'report/isaac/figures'/path.name
            destination.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(path,destination)
        markdown.extend(['![Иллюстрация](figures/'+path.name+')',caption,''])

    for index,(title,blocks) in enumerate(PAGES):
        if index: story.append(PageBreak())
        else:
            story.append(Paragraph('Воспроизведение Active Localisation<br/>в Isaac Sim',styles['title']))
            if args.draft: story.append(Paragraph('Черновик для проверки вёрстки',styles['caption']))
            story.append(Paragraph('Процесс переноса навигационного задания · 9 ноября 2026',styles['caption']))
        story.append(Paragraph(f'{index+1} {escape(title)}',styles['section']))
        markdown.extend([f'## {index+1} {title}',''])
        for kind,text in blocks:
            if kind in ('p','h'):
                para(text,kind)
            elif kind=='contract':
                table([['Компонент','Сохранённый контракт','Реализация в Isaac'],
                    ['База','cmd_vel → vx, vy, wz','Кинематическая база и swept footprint'],
                    ['Датчики','RGB/depth, CameraInfo, PointCloud2','RTX-камера и PhysX-лидары'],
                    ['Одометрия и TF','GT отдельно; noisy odom → base','Прежняя модель дрейфа'],
                    ['Локализация','map → odom, STag HD17 IDs1–5','Прежний RTAB-Map, новая карта'],
                    ['Навигация','NavigateToPose и три режима','Прежние Nav2 plugins/BT']],
                    [width*.18,width*.40,width*.42])
            elif kind=='robot':
                picture(RUNTIME/'robot/astribot_preview.png','Рис. 1. URDF-модель Astribot из исходного проекта после импорта в Isaac Sim. Фиксированная поза соответствует навигационному эксперименту.',w=width*.58)
            elif kind=='sensor':
                table([['Величина','Восстановленное значение','Фактическая проверка'],
                    ['GT / RGB+depth / lidar','50 / 30 / 10 Гц по sim time','Внешний ROS-тест частот'],
                    ['Камера','640 × 480, HFOV 1,089 рад','fx = 528,4337; optical-frame'],
                    ['Лидар','120 × 16 лучей на датчик','p99 surface error около 2 см'],
                    ['Сшивка','Два исходных облака','Прежний lidar_stitching']],
                    [width*.23,width*.39,width*.38])
            elif kind=='world':
                picture(RUNTIME/'navigation/world-verification/torso_markers.png','Рис. 3. Реальное изображение перенесённой сцены с torso-камеры. Текстуры маркеров используются детектором без передачи идеальных поз.',w=width*.64)
            elif kind=='detection':
                picture(RUNTIME/'navigation/world-verification/detections_hd17.png','Рис. 2. Исходный HD17-детектор на кадре Isaac. ID0 — контроль фронтенда, для RTAB-Map разрешены IDs1–5. FPS в оверлее — обратная средняя длительность обработки, а не частота камеры.',w=width*.51)
            elif kind=='results':
                data=[['Метрика','baseline','active','control']]
                for scale in ('4','6'):
                    data.append(['Шум ×'+scale,'','',''])
                    for key,label,factor,digits in [('ape_rmse','APE RMSE, мм',1000,1),('rpe_rmse','RPE RMSE, мм',1000,1),
                        ('det_per_m','Детекции на метр',1,1),('path_detection_pct','Путь с детекцией за 1 с, %',1,1),('duration_s','Время симуляции, с',1,1)]:
                        values=[]
                        for mode in ('baseline','active','control'):
                            s=summary['groups'][scale][mode][key]
                            values.append(f"{s['mean']*factor:.{digits}f} ± {s['std']*factor:.{digits}f}" if s['mean'] is not None and s['std'] is not None else 'нет данных')
                        data.append([label,*values])
                table(data,[width*.40,width*.20,width*.20,width*.20])
                passed=sum(r.get('goals_succeeded',0) for r in summary['rows'])
                para(f'Учтено {summary["evaluated_jobs"]} из 30 запусков; достигнуто {int(passed)} из 90 целей. Значения — среднее ± выборочное стандартное отклонение по пяти seeds. Доля пути с недавней детекцией вычислена по длине пути, а не по времени стоянки. Неудачные запуски не удалялись из учёта.','caption')
            elif kind=='conclusion':
                succeeded=int(sum(r.get('goals_succeeded',0) for r in summary['rows']))
                para(f'В Isaac Sim воспроизведены робот, собственная сцена, сенсорная локализация и исходные режимы навигации. В серии учтено {summary["evaluated_jobs"]} из 30 эпизодов; достигнуто {succeeded} из 90 целей. Валидация охватывает реальные ROS-действия и траектории, а также проверку источников данных и условий сравнения.')
            elif kind=='exceptions':
                if not args.draft:
                    para('Отдельные критерии приёмки выявили два исключения. В control ×6, seed44 конечная оценка yaw отличалась от цели на 0,1665 рад при дополнительном допуске 0,10 рад: строгую проверку позы прошли 89/90 целей. В control ×4, seed45 регистратор завершился с ошибкой после конца окна; файлы и счётчики полны. Поэтому общий runner имеет 28/30 успешных статусов при 90/90 успехах Nav2. Оба случая включены в оценку. Обработка сигналов регистратора исправлена после серии и отдельно проверена 20 остановками.')
            elif kind=='commands':
                para('Воспроизведение серии: python3 isaac/run_series.py --name UNIQUE_NAME. Одиночный эпизод: python3 isaac/run_experiment.py --batch UNIQUE_NAME --scenario shelves_loop_a --noise-scale 4 --seed 42 --modes active --complete-scenario. Все симуляционные запуски выполняются в среде Isaac; названия должны быть новыми, чтобы не перезаписывать доказательства.','caption')
    def footer(canvas,doc):
        canvas.saveState();canvas.setFont('Body',9);canvas.drawCentredString(A4[0]/2,12*mm,str(doc.page));canvas.restoreState()
    doc=SimpleDocTemplate(str(out),pagesize=A4,rightMargin=18*mm,leftMargin=18*mm,
        topMargin=16*mm,bottomMargin=18*mm,title='Воспроизведение Active Localisation в Isaac Sim',author='Active-Localisation')
    doc.build(story,onFirstPage=footer,onLaterPages=footer)
    source=out.with_suffix('.md') if args.draft else REPO/'report/isaac/reproduction.md'
    source.parent.mkdir(parents=True,exist_ok=True)
    source.write_text('\n'.join(markdown),encoding='utf-8',newline='\n')
    print(out)


if __name__=='__main__': main()
